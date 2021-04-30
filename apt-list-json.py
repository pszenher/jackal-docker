import argparse
import json

import apt


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


parser = argparse.ArgumentParser(description="Output json apt list to stdout.")
parser.add_argument(
    "-f",
    "--filter",
    type=str,
    metavar="[installed|upgradable|new]",
    help="Filter to apply to apt package list.",
)

args = parser.parse_args()

# Open system apt cache
full_cache = apt.cache.Cache()
full_cache.update()
full_cache.open()

filter_cache = apt.cache.FilteredCache(full_cache)

if args.filter is None:
    filter_cache.set_filter(CandidateFilter())
elif args.filter == "installed":
    filter_cache.set_filter(InstalledFilter())
elif args.filter == "upgradable":
    filter_cache.set_filter(UpgradableFilter())
elif args.filter == "new":
    filter_cache.set_filter(UpgradableOrNotInstalledFilter())
else:
    raise Exception()

package_dict = {pack.shortname: pack.candidate.version for pack in filter_cache}

print(json.dumps(package_dict))
