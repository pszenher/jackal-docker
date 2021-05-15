# Enable postponement of annotation evaluatoin
from __future__ import annotations

import argparse
import collections
import dataclasses
import difflib
import functools
import json
import logging
import os
import pickle
import re
import shlex
import sys
import tempfile
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, Iterable, List, Tuple, Union

# TODO: replace unneeded apt import with apt_pkg.init() call
import apt_pkg  # type: ignore
import apt_repo  # type: ignore
import docker  # type: ignore
from dockerfile_parse import DockerfileParser  # type: ignore

if sys.version_info >= (3, 8):
    from typing import TypedDict  # pylint: disable=no-name-in-module
else:
    from typing_extensions import TypedDict

apt_pkg.init()


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class PinnedPackage:
    name: str
    lineno: int
    _cur_version: Union[str, None] = None
    _candidate_versions: List[str] = dataclasses.field(default_factory=list)

    def __init__(self, package_str: str, line_num: int) -> None:
        split_str = package_str.split("=", 1)

        self.name = split_str[0]
        self.lineno = line_num
        if len(split_str) == 1:
            self._cur_version = None
        else:
            self._cur_version = split_str[1]
        self._candidate_versions = []

    @property
    def cur_version(self) -> str:
        if self._cur_version is None:
            return ""
        else:
            return self._cur_version

    @property
    def cur_string(self) -> str:
        return f"{self.name}{'=' + self._cur_version if self._cur_version is not None else ''}"

    @property
    def new_string(self) -> str:
        return f"{self.name}={self.newest_version}"

    @property
    def newest_version(self) -> str:
        if len(self._candidate_versions) == 0:
            raise RuntimeError(
                f'Package "{self.name}" has no candidate versions to generate.'
            )
        return functools.reduce(
            lambda x, y: x if apt_pkg.version_compare(x, y) > 0 else y,
            self._candidate_versions,
        )

    def is_changed(self) -> bool:
        """Return bool describing if newest candidate package version differs from current version."""
        return apt_pkg.version_compare(self.cur_version, self.newest_version) != 0

    def update_package(self, version: str) -> None:
        """Append version to package _candidate_versions list.

        Keyword Parameters:
        version -- candidate version of package
        """
        self._candidate_versions.append(version)


class PinnedPackageList:
    """List of PinnedPackage objects with helper methods for adding and updating packages."""

    packages: Dict[str, PinnedPackage]

    def __init__(
        self, packages: List[PinnedPackage] = dataclasses.field(default_factory=list)
    ):
        for package, count in collections.Counter([p.name for p in packages]).items():
            if count > 1:
                raise ValueError(
                    f'Number of instances of package "{package}"',
                    "is greater than 1 (appears {count} times.)",
                )

        self.packages = {pack.name: pack for pack in packages}

    def __iter__(self):
        yield from self.packages.values()

    def __getitem__(self, key: str) -> PinnedPackage:
        if key not in self.packages:
            raise ValueError(f'Package name "{key}" is not in the package list.')
        return self.packages[key]

    @property
    def names(self):
        """Return generator of all package names in PinnedPackageList."""
        yield from self.packages.keys()

    def add_package(self, package: PinnedPackage) -> None:
        """Add PinnedPackage to PinnedPackageList.

        Keyword Arguments:
        package -- PinnedPackage to be added
        """
        if package.name in self.packages:
            raise ValueError(
                f'Package name "{package.name}" is already in the package list.'
            )
        self.packages[package.name] = package

    def update_package(self, name: str, version: str) -> None:
        """Add candidate version to package.

        Keyword Parameters:
        name -- name of target package
        version -- new candidate version
        """
        self[name].update_package(version)


class AptDockerfileParser(DockerfileParser):
    """Subclass of DockerfilleParser for handing apt install statements in RUN commands."""

    class DockerfileCommand(TypedDict):
        """TypedDict contract class for handling typed keys in DockerfileParser command dicts."""

        instruction: str
        startline: int
        endline: int
        content: str
        value: str

    # TODO: handle multi-stage builds when parsing apt config
    @property
    def apt_pin_params(self) -> List[str]:
        # Initialize list of parameters
        repos = []

        for command in filter(
            lambda cmd: cmd["instruction"] == "COMMENT"
            and re.match(r"^\s*#\s*apt-pin:", cmd["content"]),
            self.structure,
        ):
            tokens = shlex.shlex(
                command["value"],
                posix=True,
                punctuation_chars=True,
            )
            tokens.whitespace_split = True

            # Exhaust shlex token generator
            token_list = list(tokens)

            if len(token_list) == 0:
                logging.warning(
                    "Empty apt-pin comment on line %d, ignoring", command["startline"]
                )
                continue

            logging.debug(
                "Apt-pin comment found on line %d, capturing", command["startline"]
            )
            for token in token_list[1:]:
                try:
                    key, value = token.split("=", 1)
                except ValueError as err:
                    logging.warning(
                        "Malformed apt-pin parameter %s on line %s (%s), ignoring",
                        token,
                        command["startline"],
                        err,
                    )
                    continue
                if key != "repo":
                    raise ValueError(
                        f'Unexpected apt-pin parameter key "{key}"'
                        + f'with value "{value}" on line {content["startline"]}'
                    )

                repos.append(value)

        return repos

    @property
    def apt_packages(self) -> PinnedPackageList:
        """Return PinnedPackageList containing all apt packages installed by dockerfile."""

        pkg_list = []
        for command in filter(lambda cmd: cmd["instruction"] == "RUN", self.structure):
            tokens = shlex.shlex(command["content"], posix=True, punctuation_chars=True)
            tokens.whitespace_split = True
            tokens_linum = []
            tokens.lineno -= 1
            for token in tokens:
                # TODO: fix in case of multiple consecutive comment lines (parsed as multiple commands)
                tokens.lineno += sum(
                    (
                        c["endline"] + 1 - (command["startline"] + tokens.lineno)
                        for c in self.structure
                        if c["instruction"] == "COMMENT"
                        and command["startline"] + tokens.lineno
                        in range(c["startline"], c["endline"] + 1)
                    )
                )
                tokens_linum.append(
                    (
                        token,
                        int(command["startline"]) + tokens.lineno,
                    )
                )

            pkg_list.extend(self._parse_apt_install(tokens_linum))
        return PinnedPackageList(pkg_list)

    @staticmethod
    def _parse_apt_install(tokens: Iterable[Tuple[str, int]]) -> List[PinnedPackage]:
        pkg_list = []
        last_token = ""
        is_install = False
        for token, lnum in tokens:

            # Ignore zero-length tokens (should never happen)
            if len(token) == 0:
                continue

            # Disable is_install flag upon leaving current command
            if token in ["&&", "||", ">", "<", "|", ";"]:
                is_install = False

            # Add to package dict if this is an install command and not a flag token
            elif is_install and token[0].isalnum():
                package = PinnedPackage(token, line_num=lnum)
                pkg_list.append(package)

            # Enable install flag if apt command is followed by install
            elif token == "install" and (last_token in ["apt", "apt-get"]):
                is_install = True

            # Save current token as last token
            last_token = token

        return pkg_list


def docker_get_upgradable(image: str = "jackal-kinetic") -> Dict[str, str]:
    """Return upgradable apt packages from docker image.

    Keyword Parameters:
    image -- name of docker image
    """

    docker_client = docker.from_env()
    try:
        stdout = docker_client.containers.run(
            image=image,
            command="python3 /apt-list-json.py -f new",
            user="root",
            volumes={
                sys.path[0]
                + "/apt-list-json.py": {"bind": "/apt-list-json.py", "mode": "ro"}
            },
            remove=True,
            read_only=False,
            detach=False,
            stdout=True,
        )

    except docker.errors.ContainerError as err:
        logging.error(
            'Command "%s" in image "%s" returned non-zero exit status %i:',
            err.command,
            err.image,
            err.exit_status,
        )
        logging.error(err.stderr.decode())
        sys.exit(1)

    return json.loads(stdout)


def check_cache_validity(
    cache_path: Path, force_refresh: bool, expiry_age: timedelta
) -> bool:
    """Return bool stating validity of apt package cache.

    Keyword Arguments:
    cache_path -- path of apt cache file
    force_refresh -- boolean if cache should be automatically refreshed
    expiry_age -- max age of valid cache
    """
    if not cache_path.exists():
        logging.warning(
            'Apt cache "%s" does not exist, downloading apt repo packages', cache_path
        )
        return False

    if force_refresh:
        logging.warning("Apt cache refresh requested, redownloading apt repo packages")
        return False

    cache_mtime = datetime.fromtimestamp(cache_path.stat().st_mtime)
    cache_age = datetime.now() - cache_mtime
    if cache_age > expiry_age:
        logging.warning("Apt cache expired, redownloading apt repo packages")
        return False

    logging.info("Apt cache is valid, using for update")
    return True


def main(args: argparse.Namespace) -> None:
    # Parse target dockerfile
    logging.info("Parsing dockerfile %s", args.file)
    dfp = AptDockerfileParser(args.file.as_posix())

    if args.sources_list is not None:
        with open(args.sources_list, "r") as sources_list_file:
            source_lines = sources_list_file.readlines()
    else:
        source_lines = []

    sources = apt_repo.APTSources(
        [
            apt_repo.APTRepository.from_sources_list_entry(entry)
            for entry in dfp.apt_pin_params + source_lines
        ]
    )

    # Handle apt package repo caching
    if args.cache:
        cache_path = args.cache_dir / "apt_all_packages.pickle"
        if check_cache_validity(
            cache_path=cache_path,
            force_refresh=args.refresh_cache,
            expiry_age=timedelta(hours=1),
        ):
            try:
                with open(cache_path, "rb") as cache_file:
                    all_packages = pickle.load(cache_file)
            except (pickle.UnpicklingError, EOFError) as err:
                logging.error(
                    'Could not unpickle apt cache file "%s" ("%s"), try refreshing cache',
                    cache_path,
                    err,
                )
                sys.exit(1)
            package_fetch_time = datetime.fromtimestamp(cache_path.stat().st_mtime)
        else:
            package_fetch_time = datetime.now()
            all_packages = sources.packages
            with open(cache_path, "wb") as cache_file:
                pickle.dump(all_packages, cache_file)

    else:
        logging.warning("Cache disabled, downloading apt repo packages")
        all_packages = sources.packages
        package_fetch_time = datetime.now()

    # Populate PinnedPackageList of Dockerfile apt installed packages
    docker_packages = dfp.apt_packages
    repo_packages = [
        package for package in all_packages if package.package in docker_packages.names
    ]
    for package in repo_packages:
        docker_packages.update_package(package.package, package.version)

    # Replace old lines of dockerfile with updated pinned versions
    old_lines = dfp.lines
    new_lines = dfp.lines.copy()

    for package in filter(lambda p: p.is_changed(), docker_packages):
        new_lines[package.lineno], num_sub = re.subn(
            fr"(\s|\"){re.escape(package.cur_string)}(\s|\"|\\)",
            fr"\1{package.new_string}\2",
            old_lines[package.lineno],
        )
        if num_sub != 1:
            raise RuntimeError(
                f"Expected 1 replacement on line {package.lineno+1}",
                f"from {package.cur_string} to {package.new_string},",
                f"got: {num_sub}.",
                f"line: {old_lines[package.lineno]}",
            )
        logging.info(
            "%-40s --> %s",
            package.cur_string,
            package.new_string,
        )

    if args.write:
        # Write lines to DockerfileParser setter, writing to dockerfile
        dfp.lines = new_lines
    else:
        sys.stdout.writelines(new_lines)

    if args.diff:
        # Generate diff of old and new lines
        diff = list(
            difflib.unified_diff(
                old_lines,
                new_lines,
                fromfile=args.file.name,
                tofile="apt repositories",
                fromfiledate=datetime.fromtimestamp(
                    args.file.stat().st_mtime
                ).isoformat(),
                tofiledate=package_fetch_time.isoformat(),
            )
        )

        for find, replace in [
            (r"^(\+{3}.*|-{3}.*)$", fr"{bcolors.BOLD}\1{bcolors.ENDC}"),
            (r"^(\@{2}.*)$", fr"{bcolors.OKBLUE}\1{bcolors.ENDC}"),
            (r"^(\+\s{4}.*)$", fr"{bcolors.OKGREEN}\1{bcolors.ENDC}"),
            (r"^(-\s{4}.*)$", fr"{bcolors.FAIL}\1{bcolors.ENDC}"),
        ]:
            diff = [re.sub(find, replace, line) for line in diff]

        sys.stderr.writelines(diff)


if __name__ == "__main__":
    _parser = argparse.ArgumentParser(
        description="Update dockerfile pinned apt versions to latest version."
    )
    _parser.add_argument(
        "-f",
        "--file",
        required=True,
        type=Path,
        action="store",
        help="path to dockerfile to update",
    )
    _parser.add_argument(
        "-w",
        "--write",
        default=False,
        action="store_true",
        help="write result to file instead of stdout",
    )
    _parser.add_argument(
        "-d",
        "--diff",
        default=False,
        action="store_true",
        help="print diff of Dockerfile to stderr",
    )
    _parser.add_argument(
        "--cache",
        dest="cache",
        default=True,
        action="store_true",
        help="enable caching of apt repositories (default: %(default)s)",
    )
    _parser.add_argument(
        "--no-cache",
        dest="cache",
        action="store_false",
        help="disable apt repositories caching",
    )
    _parser.add_argument(
        "--refresh-cache",
        default=False,
        action="store_true",
        help="force refresh apt repository cache",
    )
    _parser.add_argument(
        "--cache-dir",
        default=tempfile.gettempdir(),
        type=Path,
        metavar="CACHE",
        action="store",
        help="directory to store apt cache in (default %(default)s)",
    )
    # TODO: use this argument for apt_repo sources
    _parser.add_argument(
        "--sources-list",
        type=Path,
        metavar="FILE",
        action="store",
        help="path to apt sources.list file containing apt repos",
    )

    _args = _parser.parse_args()

    logging.getLogger().setLevel(logging.WARN)
    main(_args)
