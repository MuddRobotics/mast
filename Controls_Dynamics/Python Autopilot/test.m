if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end



autopilot = py.autopilot.makeAutopilot();