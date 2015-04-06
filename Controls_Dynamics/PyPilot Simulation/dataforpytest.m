pytestinputsData = pytestinputs.Data;
pytestinputsTime = pytestinputs.Time;
pytestoutputsData = pytestoutputs.Data;
pytestoutputsTime = pytestoutputs.Time;

csvwrite('PyTestInputs.csv',[pytestinputsTime,pytestinputsData]);
csvwrite('PyTestOutputs.csv',[pytestoutputsTime,pytestoutputsData]);
