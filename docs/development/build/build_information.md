# Fuchsia Build Information

We collection metrics and error reports from devices in a few ways: Cobalt, feedback reports, crashpad crashes, manual reports from developers and QA.  Interpreting these signals requires knowing where they are generated from to varying levels of detail.  This document describes the places where version information about the system are stored for use in these types of reports.  Note that this information only applies to the base system - dynamically or ephemerally added software will not be included here.

To access this data, add the feature "build-info" to the [component manifest](../../the-book/package_metadata.md#component-manifest) of the component that needs to read these fields.


# Product
## Location
`/config/build-info/product`

## Description
String describing the product configuration used at build time.  Defaults to the value passed as PRODUCT in fx set.
Example: “products/core.gni”, “products/workstation.gni”

# Board
## Location
`/config/build-info/board`

## Description
String describing the board configuration used at build time to specify the target hardware.  Defaults to the value passed as BOARD in fx set.
Example: “boards/x64.gni”

# Version
## Location
`/config/build-info/version`

## Description
String describing the version of the build.  Defaults to the same string used currently in ‘latest-commit-date’.  Can be overridden by build infrastructure to provide a more semantically meaningful version, e.g. to include the release train the build was produced on.

# Latest-commit-date
## Location
`/config/build-info/latest-commit-date`

## Description
String containing a timestamp of the most recent commit to the integration repository (specifically, the "CommitDate" field) formatted in strict ISO 8601 format in the UTC timezone.  Example: “2019-03-28T15:42:20+00:00”.

# Snapshot
## Location:
`/config/build-info/snapshot`

## Description
Jiri snapshot of the most recent ‘jiri update’

# Kernel version
## Location:
Stored in vDSO.  Accessed through [`zx_system_get_version`]( /zircon/docs/syscalls/system_get_version.md)

Zircon revision computed during the kernel build process.
