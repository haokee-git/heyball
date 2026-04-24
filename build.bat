@echo off
setlocal

set "ROOT=%~dp0"
set "CMAKE=D:\msys64\ucrt64\bin\cmake.exe"

pushd "%ROOT%"

echo Configuring Debug...
"%CMAKE%" --preset ucrt64-debug
if errorlevel 1 goto fail

echo Building Debug...
"%CMAKE%" --build --preset ucrt64-debug
if errorlevel 1 goto fail

echo Configuring Release...
"%CMAKE%" --preset ucrt64-release
if errorlevel 1 goto fail

echo Building Release...
"%CMAKE%" --build --preset ucrt64-release
if errorlevel 1 goto fail

echo Build completed.
echo Debug:   build\ucrt64-debug\heyball.exe
echo Release: build\ucrt64-release\heyball.exe

popd
exit /b 0

:fail
popd
exit /b 1
