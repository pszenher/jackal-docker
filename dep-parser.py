# Enable postponement of annotation evaluatoin
from __future__ import annotations

import fileinput
import json
import logging
import shlex
import sys
from dataclasses import dataclass, field
from pprint import pprint
from typing import Dict, Iterable, List, Optional, Union

import docker
from dockerfile_parse import DockerfileParser


@dataclass
class Package:
    name: str
    version: Union[str, None] = None
    upgrade_version: Union[str, None] = None

    def gen_apt_string(self) -> str:
        return f"{self.name}{self._get_version_string()}"

    def gen_upgrade_string(self) -> str:
        if self.upgrade_version is None:
            raise RuntimeError(
                f'Package "{self.name}" has no upgrade_version to generate.'
            )
        return f"{self.name}{self._get_upgrade_string()}"

    def is_upgradable(self) -> bool:
        return self.upgrade_version is not None

    def _get_version_string(self) -> str:
        return "=" + self.version if self.version is not None else ""

    def _get_upgrade_string(self) -> str:
        return "=" + self.upgrade_version if self.upgrade_version is not None else ""

    @staticmethod
    def parse_apt_string(package_str: str) -> Package:
        split_str = package_str.split("=", 1)
        if len(split_str) == 1:
            return Package(name=split_str[0], version=None)
        return Package(name=split_str[0], version=split_str[1])

    @staticmethod
    def parse_apt_dict(package_dict: Dict[str, str]) -> Dict[str, Package]:
        return {n: Package(name=n, version=v) for n, v in package_dict.items()}


@dataclass
class PackageList:
    packages: Dict[str, Package] = field(default_factory=dict)

    def add_package(self, package: Package) -> None:
        if package.name in self.packages:
            raise ValueError(
                f'Package name "{package.name}" is already in the package list.'
            )
        self.packages[package.name] = package
        return

    def add_packages(self, new_package_list: PackageList) -> None:
        for package in new_package_list.packages.values():
            self.add_package(package)
        return

    def upgrade_package(self, name: str, version: str) -> None:
        if name not in self.packages:
            raise ValueError(f'Package name "{name}" is not in the package list.')
        self.packages[name].upgrade_version = version
        return


class AptDockerfileParser(DockerfileParser):
    def apt_packages(self) -> PackageList:
        run_commands = [
            cmd["value"] for cmd in self.structure if cmd["instruction"] == "RUN"
        ]

        packages = PackageList()
        for command in run_commands:
            tokens = shlex.shlex(command, posix=True, punctuation_chars=True)
            tokens.whitespace_split = True
            packages.add_packages(self._parse_apt_install(tokens))

        return packages

    @staticmethod
    def _parse_apt_install(tokens: Iterable[str]) -> PackageList:
        pkg_list = PackageList()
        last_token = ""
        is_install = False
        for token in tokens:

            # Ignore zero-length tokens (should never happen)
            if len(token) == 0:
                continue

            # Disable is_install flag upon leaving current command
            if token in ["&&", "||", ">", "<", "|", ";"]:
                is_install = False

            # Add to package dict if this is an install command and not a flag token
            elif is_install and token[0].isalnum():
                package = Package.parse_apt_string(token)
                pkg_list.add_package(package)

            # Enable install flag if apt command is followed by install
            elif token == "install" and (
                last_token == "apt-get" or last_token == "apt"
            ):
                is_install = True

            # Save current token as last token
            last_token = token

        return pkg_list


if __name__ == "__main__":

    # Parse target dockerfile
    logging.warning("Parsing dockerfile...")
    dfp = AptDockerfileParser()
    with open("jackal-kinetic.Dockerfile", "r") as f:
        dfp.content = f.read()

    packages = dfp.apt_packages()

    docker_client = docker.from_env()
    stdout = docker_client.containers.run(
        image="jackal-kinetic",
        command="python3 /apt-list-json.py -f new",
        user="root",
        volumes={
            sys.path[0]
            + "/apt-list-json.py": {"bind": "/apt-list-json.py", "mode": "ro"}
        },
        auto_remove=True,
        read_only=False,
        detach=False,
        stdout=True,
    )

    upgradable = json.loads(stdout)

    for n, v in upgradable.items():
        try:
            packages.upgrade_package(n, v)
        except ValueError:
            # Ignore upgradable packages that are not listed in the dockerfile
            pass

    for p in packages.packages.values():
        if p.is_upgradable():
            print(p.gen_upgrade_string())

    # # Perform inplace replacement of old version number in dockerfile
    # filename = "jackal-kinetic.Dockerfile"
    # with fileinput.FileInput(filename, inplace=True, backup=".bak") as file:
    #     for line in file:
    #         for change in changes:
    #             print(
    #                 line.replace(
    #                     f"{change.name}={change.old_version}",
    #                     f"{change.name}={change.version}",
    #                 ).rstrip("\n")
    #             )
