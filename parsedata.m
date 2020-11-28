function data = parsedata(rawdata)
    split_rawdata = str2double(split(rawdata));
    data = split_rawdata(2:end-2);
end