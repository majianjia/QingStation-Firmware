# Change Log
All notable changes to the BMM150 Sensor API will be documented in this file.

## v1.0.3, 10 Jan 2020
### Changed
- Updated License 

## v1.0.2, 23 Dec 2019
### Changed
- Fixed warnings and errors from static code analysis tools 

## v1.0.1, 08 Nov 2018
### Changed
- Auto-aligned the Source code using uncrustify 

## v0.9.0, 12 Mar 2017
### Added
- Aux. Data compensation API to support when BMM150 is connected as auxiliary sensor
### Fixed
- All PC-Lint infos related to shifting of zero fixed.

## v0.8.0, 1 Mar 2017
### Added
- Interrupt support added for BMM150 API - Data ready, low threshold , high threshold overflow and overrun

## v0.7.0, 17 Feb 2017
### Changed
- Self test result is returned as return value instead of using a parameter to the API
### Added
- BMM150 warning codes.

## v0.6.0, 15 Feb 2017
### Changed
- Shifting of integers in compensation equations and data read is replaced with multiplication/division.
- Usage of "BMM150_USE_FLOATING_POINT" macro and data read modified.
### Fixed
- All PC-Lint infos related to shifting of integers fixed.