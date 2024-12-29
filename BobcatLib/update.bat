@echo off
echo installing roboRIOToolchain
call gradlew.bat installRoboRIOToolchain
echo Formatting
call gradlew.bat :spotlessApply
echo Building
call gradlew.bat build
echo Publish
call gradlew.bat publish
echo Copy
Xcopy /E /y .\\build\\repos\\releases\\BobcatLib .\\BobcatLib\\repos\\BobcatLib
