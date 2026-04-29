function params = helperNedParams(mapOrigin)
params = struct('Frame','NED','IsCartesian',true,'AxesOrder','interleaved','OriginPosition',mapOrigin);
end