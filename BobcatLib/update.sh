#!/bin/bash
./gradlew :spotlessApply
./gradlew build
./gradlew installRoboRIOToolchain
./gradlew publish
cp -rf build/repos/releases/BobcatLib BobcatLib/repos