#!/bin/bash
./gradlew installRoboRIOToolchain
./gradlew :spotlessApply
./gradlew build
./gradlew publish
cp -rf build/repos/releases/BobcatLib BobcatLib/repos/
