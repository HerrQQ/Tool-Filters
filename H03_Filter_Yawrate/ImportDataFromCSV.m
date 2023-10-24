function ImportData()
clc
clear
[file,path]=uigetfile(pwd,"*.csv","read csv");

[DataList,~,~] = xlsread(file);
DeltaSout = DataList(:,1);
ts=size(DeltaSout)
% DeltaS = DataList(:,2);
TimeList=linspace(1,900,ts(1));
tsDeltaSout=timeseries(DeltaSout,TimeList);
assignin('base','tsDeltaS',tsDeltaSout);

rackAngle = DataList(:,2);
tsrackAngle=timeseries(rackAngle,TimeList);
assignin('base','tsdelatCnt',tsrackAngle);

DeltaS = DataList(:,3);
tsDeltaS=timeseries(DeltaS,TimeList);
assignin('base','tsDeltaSout',tsDeltaS);

enableCnt = DataList(:,4);
tsenableCnt=timeseries(enableCnt,TimeList);
assignin('base','tsrackAngle',tsenableCnt);

delatCnt = DataList(:,5);
tsdelatCnt=timeseries(delatCnt,TimeList);
assignin('base','tsdelatCntRack',tsdelatCnt);

yawrate = DataList(:,6);
tsyawrate=timeseries(yawrate,TimeList);
assignin('base','tsrackAngleout',tsyawrate);

Ay = DataList(:,7);
tsAy=timeseries(Ay,TimeList);
assignin('base','tsAy',tsAy);

steeringAngle = DataList(:,8);
tssteeringAngle=timeseries(steeringAngle,TimeList);
assignin('base','tssteeringAngle',tssteeringAngle);

steeringTorque = DataList(:,9);
tssteeringTorque=timeseries(steeringTorque,TimeList);
assignin('base','tsenCnt',tssteeringTorque);

deltaWheelSpeed = DataList(:,10);
tsdeltaWheelSpeed=timeseries(deltaWheelSpeed,TimeList);
assignin('base','tsenCntRack',tsdeltaWheelSpeed);

end