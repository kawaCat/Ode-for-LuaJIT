
--====================================================================
local ffi =require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds =require "drawstuff"
--====================================================================

-- drawstuff simlation loop func
--====================================================================
function simLoop (pause)
end
--====================================================================

-- drawstuff start func
--====================================================================
function start()
end
--====================================================================

--====================================================================
function stop()
end 
--====================================================================

--====================================================================
function drawGeom( g)
end 
--====================================================================

--====================================================================
function main ()
    --================================================================
    ode.dInitODE();
    --================================================================
    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = nil;      -- nothing func so nil
    func[0].stop  = stop;        
    func[0].path_to_textures = "./textures";

    --drawstuff
    ds.dsSimulationLoop (0,nil,352,288,func);

    -- destroy world
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================
