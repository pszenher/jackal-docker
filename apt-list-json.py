import argparse
import json

import apt  # type: ignore


class CandidateFilter(apt.cache.Filter):
    @staticmethod
    def apply(pkg: apt.package.Package) -> bool:
        return pkg.candidate is not None


class UpgradableFilter(apt.cache.Filter):
    @staticmethod
    def apply(pkg: apt.package.Package) -> bool:
        return pkg.is_upgradable


class InstalledFilter(apt.cache.Filter):
    @staticmethod
    def apply(pkg: apt.package.Package) -> bool:
        return pkg.is_installed


class UpgradableOrNotInstalledFilter(apt.cache.Filter):
    @staticmethod
    def apply(pkg: apt.package.Package) -> bool:
        return pkg.is_upgradable or not pkg.is_installed


def apt_filter(filter_name: str):
    if args.filter is None:
        return CandidateFilter()
    elif args.filter == "installed":
        return InstalledFilter()
    elif args.filter == "upgradable":
        return UpgradableFilter()
    elif args.filter == "new":
        return UpgradableOrNotInstalledFilter()
    else:
        raise ValueError(f'Filter name "{filter_name}" is not a valid filter.')


def main(args: argparse.Namespace) -> None:
    # Open system apt cache
    full_cache = apt.cache.Cache()
    full_cache.update()
    full_cache.open()

    filter_cache = apt.cache.FilteredCache(full_cache)
    filter_cache.set_filter(args.apt_filter)

    package_dict = {pack.shortname: pack.candidate.version for pack in filter_cache}

    print(json.dumps(package_dict))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Output json apt list to stdout.")
    parser.add_argument(
        "-f",
        "--filter",
        type=apt_filter,
        metavar="[installed|upgradable|new]",
        help="Filter to apply to apt package list.",
    )
    args = parser.parse_args()

    main(args)
