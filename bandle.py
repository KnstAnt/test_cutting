import os
import re

def clean_rust_code(code):
    # Удаляем многострочные комментарии /* ... */
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.DOTALL)
    # Удаляем однострочные комментарии // ...
    code = re.sub(r'//.*', '', code)
    # Удаляем пустые строки и лишние пробелы в начале/конце
    lines = [line.rstrip() for line in code.splitlines() if line.strip()]
    return "\n".join(lines)

def resolve_module(base_path, mod_name):
    # Пути, где Rust ищет модули
    variants = [
        os.path.join(base_path, f"{mod_name}.rs"),
        os.path.join(base_path, mod_name, "mod.rs")
    ]
    for v in variants:
        if os.path.exists(v):
            return v
    return None

def bundle(file_path):
    if not os.path.exists(file_path):
        return f"// Error: {file_path} not found"

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    base_dir = os.path.dirname(file_path)
    
    # Регулярка для поиска 'mod name;' (не трогает 'mod name { ... }')
    mod_pattern = re.compile(r'^(\s*)mod\s+([a-zA-Z0-9_]+)\s*;', re.MULTILINE)
    
    def replace_mod(match):
        indent = match.group(1)
        mod_name = match.group(2)
        
        mod_file = resolve_module(base_dir, mod_name)
        if mod_file:
            inner_code = bundle(mod_file)
            return f"{indent}mod {mod_name} {{\n{inner_code}\n{indent}}}"
        return match.group(0) # Если файл не найден (например, внешняя либа), оставляем как есть

    bundled_code = mod_pattern.sub(replace_mod, content)
    return bundled_code

def main():
    input_file = 'src/main.rs'  # Укажите путь к вашему main.rs
    output_file = 'bundled_main.rs'

    if not os.path.exists(input_file):
        print(f"Файл {input_file} не найден. Запустите скрипт из корня проекта.")
        return

    print("Сборка проекта...")
    full_code = bundle(input_file)
    clean_code = clean_rust_code(full_code)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(clean_code)
    
    print(f"Готово! Файл сохранен в: {output_file}")

if __name__ == "__main__":
    main()
