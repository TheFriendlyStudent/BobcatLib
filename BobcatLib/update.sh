#!/bin/bash
./gradlew :spotlessApply
./gradlew build
./gradlew installRoboRIOToolchain
./gradlew publish
