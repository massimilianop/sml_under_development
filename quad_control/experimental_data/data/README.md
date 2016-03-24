# Instructions

1. Run
```
file_name_to_plot=$(ls -t *.txt | head -n1)
matlab -nodisplay -r 'print_html '$file_name_to_plot'; exit;'
file_name_to_plot=${file_name_to_plot%".txt"}
echo '*\n!.gitignore' >$file_name_to_plot/.gitignore
unset file_name_to_plot 
```
