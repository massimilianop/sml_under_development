# Instructions

1. Run
```
file_name_to_plot=$(ls -t *.txt | head -n1); matlab -nodisplay -r 'print_html '$file_name_to_plot'; exit;'
```
