# Instructions

1. Run
```
file_name_to_plot=$(ls -t *.txt | head -n1)
matlab -nodisplay -r 'print_html '$file_name_to_plot'; exit;'
file_name_to_plot=${file_name_to_plot%".txt"}
echo '*\n!.gitignore' >$file_name_to_plot/.gitignore
unset file_name_to_plot 
```

```
# find earliest txt file
file_name_to_plot=$(ls -t *.txt | head -n1)
# matlab to publish data in txt file
matlab -nodisplay -r 'print_html '$file_name_to_plot'; exit;'
# remove txt extension from file_name_to_plot
file_name_to_plot=${file_name_to_plot%".txt"}
# include gitignore in created file, so that it is not tracked by git
echo '*\n!.gitignore' >$file_name_to_plot/.gitignore
# clear variable
unset file_name_to_plot 
```

