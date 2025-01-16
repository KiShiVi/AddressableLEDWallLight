import sys

def html_to_c_string(html_file, c_file):
    with open(html_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    with open(c_file, 'w', encoding='utf-8') as f:
        f.write('const char html_page[] =\n')
        for line in lines:
            # Убираем символы переноса строк, экранируем обратные слеши и кавычки
            escaped_line = line.rstrip().replace('\\', '\\\\').replace('"', '\\"')
            f.write(f'    "{escaped_line}\\n"\n')
        f.write(';\n')

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python html_to_c.py input.html output.c")
        sys.exit(1)

    html_file = sys.argv[1]
    c_file = sys.argv[2]

    html_to_c_string(html_file, c_file)
    print(f"Generated C string variable 'html_page' in '{c_file}'")
