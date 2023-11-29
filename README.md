# robot-2023

[![Build](https://github.com/goldenhornfrc/2023-offseason-rewrite/actions/workflows/build.yml/badge.svg)](https://github.com/goldenhornfrc/2023-offseason-rewrite/actions/workflows/build.yml)
[![Format](https://github.com/goldenhornfrc/2023-offseason-rewrite/actions/workflows/format.yml/badge.svg)](https://github.com/goldenhornfrc/2023-offseason-rewrite/actions/workflows/format.yml)

**Formatting with pre-commit**
If running pre-commit for the first time, run command:

    pre-commit install

The formatting should be done by using the command:

    pre-commit run -a


**Formatting with spotless**

The formatting of the Java code will be checked with every CI build. Run the following command to fix formatting errors:

    ./gradlew spotlessApply
