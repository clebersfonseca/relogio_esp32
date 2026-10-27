import os

Import("env")

env_file = os.path.join(env.get("PROJECT_DIR"), ".env")

if os.path.exists(env_file):
    with open(env_file) as f:
        for line in f:
            line = line.strip()
            # Ignora linhas vazias ou comentários
            if not line or line.startswith("#") or "=" not in line:
                continue
            
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip().strip("'\"")
            
            # Injeta a macro como string C++: -DKEY=\"VAL\"
            env.Append(CPPDEFINES=[(key, f'\\"{val}\\"')])
else:
    print("\n[AVISO] Arquivo .env não encontrado! Usando valores padrão ou falhará na compilação.\n")