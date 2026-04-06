# Behaviour Tuner documentation

This application tunes a morpheme network using a script and reference network running in a Squirrel enabled runtime executable.

Arguments:
- **-tuningConfigurationFile FILE** : Uses FILE to define the behaviour. Defaults to tuning.xml.
- **-continueFromLastRun** : Continues tuning by loading the state from the tuning configuration file. Note that the genetic algorithm population size and number of tuneables must match.
- **-verbose** : Enable verbose logging.
- **-startTime X** : Waits until X before tuning. Takes military time. (e.g. 20.5 for 8:30pm)
- **-endTime X** : Stops tuning after X. Takes military time. (e.g. 13.75 for 1:45pm)
- **maxNumProcesses NUM** : Use NUM threads. Note that if this is not 1 (default) then it must be an even number.