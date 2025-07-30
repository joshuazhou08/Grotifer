I have implemented a timeToRunFlag and a SetTimeToRunFlag() function in the BaseTask. It should be implemented at the beginning of the attitude control task to allow it to be controlled by TaskCoordinate task. 

I have written the TaskCoordinate task to require a few functions/flags -- 

GetFindingSunDoneFlag() function -- bool GetFindingSunDoneFlag() {return doneFindingSunFlag;} and associated flag in the AttitudeControl task to know when to start moving the torp arms

SetStartMovingFlag() function -- void SetStartMovingFlag(bool flagVal) {startMovingFlag = flagVal;} and associated flag to start MOVING to commanded position after torp masses deployed

GetDoneMovingFlag() function -- bool GetDoneMovingFlag()  {return doneMovingFlag;} and associated flag to know when to start retracting the torp masses. 

I implemented these into the partial attitude control task that was pushed to the repo when I started to work on the torp tasks so that I could check compiling errors. There are no errors in this branch.