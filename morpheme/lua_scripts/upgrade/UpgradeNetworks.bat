@echo off
echo -----------------------------------------------------------
echo -- 
echo -- This batch file will update all the .mcn files
echo -- in a set location, to the current version of Morpheme.
echo -- 
echo -- You can change the folder the lua script loads .mcn from 
echo -- in:"scripts/upgrade/updateMcn.lua"
echo --
echo -- Please Contact NaturalMotion Morpheme support
echo -- if you have any problems with this process.
echo --
echo -----------------------------------------------------------

setlocal

call ..\..\variables.bat

:PROCEED
echo **Make a backup of your old files before proceeding**
echo **Ensure write protection is disabled on the .mcn files**
echo **This process will update and overwrite old .mcn files**

set /p yn=Continue [y/n]? 

if "%yn%"=="" (echo Invalid response) & (GOTO PROCEED)
if /I "%yn%"=="n" (GOTO END)
if /I "%yn%"=="y" (GOTO UPDATE) else (echo Invalid response) & (GOTO PROCEED)

:UPDATE
echo -- Starting %APP_EXE_NAME% in command line mode
echo -- Please wait for process to complete this may take
echo -- several minutes depending on the number of files
echo -- in the library.
echo --

"%APP_EXE%" -nogui -script "%INSTDIR%/scripts/upgrade/UpdateMcn.lua"

echo -- Process complete. Opening log.
START %APP_LOG%
PAUSE
EXIT

:END
echo -- Batch file will now exit. No files have been updated
PAUSE
EXIT

