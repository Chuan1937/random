function str = msg(msgID,lang)


narginchk(1,2)

if nargin == 1
    lang = getpref('ParabolaApp','lang','en');
end

data = readtable(fullfile(fileparts(mfilename('fullpath')),'language_table.xlsx'),'ReadRowNames',true,'Basic',true);

lang = validatestring(lang,data.Properties.VariableNames);

msgID = validatestring(msgID,data.Properties.RowNames);

str = data.(lang){msgID};

end