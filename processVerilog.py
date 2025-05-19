import re
import os

def remove_comments_keep_newlines(verilog_code):
    # Remove multi-line comments (/* ... */)
    code_no_multiline_comments = re.sub(r'/\*.*?\*/', '', verilog_code, flags=re.DOTALL)
    # Remove single-line comments (//...)
    code_no_singleline_comments = re.sub(r'//.*', '', code_no_multiline_comments)
    return code_no_singleline_comments

def process_file(filename):
    if not filename.endswith(".v"):
        print(f"Skipping non-verilog file: {filename}")
        return
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            verilog_code = f.read()
        cleaned_code = remove_comments_keep_newlines(verilog_code)
        output_filename = os.path.splitext(filename)[0] + ".txt"
        with open(output_filename, 'w', encoding='utf-8') as f:
            f.write(cleaned_code)
        print(f"Processed {filename} -> {output_filename}")
    except Exception as e:
        print(f"Failed to process {filename}: {e}")

if __name__ == "__main__":
    filenames = input("Enter Verilog filenames (separated by space): ").split()
    for file in filenames:
        process_file(file)

