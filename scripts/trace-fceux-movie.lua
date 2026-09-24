-- Reference movie trace through FCEUX's public Lua API.
local movie_path = assert(os.getenv("CUPID_REFERENCE_MOVIE"))
local prefix = assert(os.getenv("CUPID_REFERENCE_PREFIX"))
assert(movie.play(movie_path, true, 0), "Could not open the supplied movie")
local total = movie.length()
local ram = assert(io.open(prefix .. ".ram", "wb"))
local trace = assert(io.open(prefix .. ".csv", "w"))
trace:write("frame,lag_count,lagged,pc\n")
emu.speedmode("nothrottle")
-- Lua can first resume before input row zero. Disable the optional end pause
-- so the final completed frame is observed by the next Lua resume as well.
emu.registerafter(function() emu.unpause() end)
for frame = 1, total do
    repeat emu.frameadvance() until emu.framecount() >= frame
    assert(emu.framecount() == frame, "Movie cursor skipped a frame")
    ram:write(memory.readbyterange(0, 0x800))
    trace:write(string.format("%d,%d,%d,%d\n", frame, emu.lagcount(),
        emu.lagged() and 1 or 0, memory.getregister("pc")))
end
ram:close()
trace:close()
local pixels = assert(io.open(prefix .. ".pixels", "wb"))
for y = 0, 239 do
    local row = {}
    for x = 0, 255 do
        local _, _, _, index = emu.getscreenpixel(x, y, true)
        row[#row + 1] = string.char(index % 64)
    end
    pixels:write(table.concat(row))
end
pixels:close()
local result = assert(io.open(prefix .. ".result", "w"))
result:write(string.format("%d\n", total))
result:close()
emu.exit()
