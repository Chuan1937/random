function [langCode, langName] = languages()


data = readtable(fullfile(fileparts(mfilename('fullpath')),'language_table.xlsx'),'ReadRowNames',true,'Basic',true);

langCode = data.Properties.VariableNames;
langName = data{'LanguageName',:};

end