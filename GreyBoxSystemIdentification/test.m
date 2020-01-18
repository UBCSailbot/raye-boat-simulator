prompt   = {'Input: row #, section #, image quantity, vine distance'};
name     = 'Input';
numlines = 4;
dlg_ans  = inputdlg(prompt, name, numlines);

dlg_ans{1}(1,:)
dlg_ans{1}(2,:)
dlg_ans{1}(3,:)
dlg_ans{1}(4,:)