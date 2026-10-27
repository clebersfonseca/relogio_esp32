# Relógio ESP32-S3 (AMOLED 2.16" + Gemini)

Firmware de um relógio inteligente touch para a placa **Waveshare ESP32-S3-Touch-AMOLED-2.16**, feito com PlatformIO + Arduino + LVGL 8.

## O que o projeto faz

- Relógio com hora sincronizada por NTP (fuso UTC-3, servidores `ntp.br`).
- Previsão do tempo (Open-Meteo), com localização automática por IP (ip-api.com) ou manual.
- Interface touch em LVGL (tela 480x480).
- Nível de bolha usando o acelerómetro/giroscópio QMI8658.
- Assistente com **Gemini**: texto, microfone (codec ES8311) e voz (TTS) pelo alto-falante.

> Ideia futura: um app/extensão de navegador que conte o consumo do plano Gemini Pro e exiba o resultado no relógio. Não existe API oficial para isso na assinatura do navegador; ver "Roadmap".

## Hardware suportado

| Item | Detalhe |
|------|---------|
| Placa | Waveshare **ESP32-S3-Touch-AMOLED-2.16** |
| MCU | ESP32-S3, 16 MB flash (QIO), PSRAM octal (OPI) |
| Ecrã | AMOLED 2.16" 480x480, driver CO5300, via QSPI |
| Touch | CSTXXX (I2C, endereço `0x5A`) |
| IMU | QMI8658 |
| Áudio | Codec ES8311 + microfones |

O `platformio.ini` usa a board genérica `esp32-s3-devkitc-1`, com flash/PSRAM configurados para esta placa. Outra placa exigiria ajustar a pinagem em [src/main.cpp](src/main.cpp) (pinos LCD, I2C, I2S) e a configuração de flash/PSRAM.

## Pré-requisitos

- [VS Code](https://code.visualstudio.com/) com a extensão **PlatformIO IDE**, ou o [PlatformIO Core (CLI)](https://platformio.org/install/cli).
- Cabo USB-C com dados.
- Rede Wi-Fi 2.4 GHz.
- Chave de API do Gemini (gratuita em <https://aistudio.google.com/apikey>), necessária apenas para o assistente.

## Configuração

Copie o modelo [contrib/.env.sample](contrib/.env.sample) para a raiz com o nome `.env` e preencha os valores (o `.env` já está no `.gitignore`, não o envie ao Git):

```bash
cp contrib/.env.sample .env
```

| Variável | Descrição | Padrão |
|----------|-----------|--------|
| `WIFI_SSID` / `WIFI_PASS` | Rede Wi-Fi 2.4 GHz (obrigatório) | - |
| `GMT_OFFSET_SEC` | Fuso horário em segundos | `-10800` (UTC-3) |
| `DAYLIGHT_OFFSET_SEC` | Horário de verão em segundos | `0` |
| `NTP_SERVER1..3` | Servidores NTP | `a.st1.ntp.br`, `b.st1.ntp.br`, `pool.ntp.org` |
| `NTP_SYNC_INTERVAL_MS` | Intervalo de ressincronização | `3600000` (1 h) |
| `LATITUDE` / `LONGITUDE` / `CITY_NAME` | Local do tempo; vazio = detetar por IP | vazio |
| `GEMINI_API_KEY` | Chave da API Gemini | - |
| `GEMINI_MODEL` | Modelo de texto | `gemini-3.5-flash-lite` |
| `GEMINI_TTS_MODEL` | Modelo de voz (TTS) | `gemini-3.1-flash-tts-preview` (padrão do código; o modelo `.env.sample` sugere `gemini-tts-1.0`) |

O script [load_env.py](load_env.py) lê o `.env` antes da compilação e transforma cada linha numa macro C++ (`-DCHAVE="valor"`). Variáveis omitidas usam os padrões de [src/main.cpp](src/main.cpp). Sem `.env`, o Wi-Fi não conecta.

## Build e upload

Com a CLI do PlatformIO:

```bash
pio run                      # compilar
pio run -t upload            # gravar na placa
pio device monitor           # monitor serial (115200)
```

Na extensão do VS Code, use os botões **Build**, **Upload** e **Monitor** da barra inferior.

Passo a passo:

1. Clone/abra a pasta do projeto.
2. Copie `contrib/.env.sample` para `.env` e preencha (ver acima).
3. Ligue a placa por USB-C.
4. Execute `pio run -t upload`. Na primeira vez, o PlatformIO baixa a plataforma `espressif32` e as bibliotecas (`lvgl`, `SensorLib`, `ArduinoJson`).
5. Abra o monitor serial para ver os logs.

### Problemas comuns

- **A placa não é detetada / falha ao gravar:** segure o botão **BOOT**, toque em **RESET** e solte o BOOT para entrar em modo de download. Depois repita o upload. No Linux, adicione o seu utilizador ao grupo `dialout` (`sudo usermod -aG dialout $USER`) e reinicie a sessão.
- **Sem saída no monitor serial:** o projeto usa USB CDC (`ARDUINO_USB_CDC_ON_BOOT=1`); reabra o monitor após o reset.
- **Erro HTTP do Gemini:** confira `GEMINI_API_KEY` e `GEMINI_MODEL` no `.env`.
- **Alterou o `.env` e nada mudou:** rode `pio run -t clean` e compile de novo.

## Estrutura

```
contrib/.env.sample  Modelo de configuração (copiar para a raiz como .env)
platformio.ini     Configuração da board, flags e dependências
default_16MB.csv   Tabela de partições (2 apps OTA de 6,25 MB + SPIFFS)
load_env.py        Injeta o .env como macros de compilação
src/main.cpp       Firmware (UI, Wi-Fi, NTP, tempo, IMU, áudio, Gemini)
src/lv_conf.h      Configuração do LVGL
src/bg_racing.c    Imagem de fundo
```

## Segurança

- Nunca faça commit do `.env`: ele contém a senha do Wi-Fi e a chave do Gemini.
- A chave compilada fica dentro do firmware; não distribua o binário.

## Roadmap: consumo do plano Gemini Pro

O consumo da assinatura usada no navegador não tem API pública. Caminho viável:

1. Extensão de navegador (Chrome/Firefox) que conta os prompts enviados em gemini.google.com, contra um limite que você configura.
2. A extensão envia o total para um pequeno servidor local.
3. O ESP32-S3 consulta esse servidor via HTTP e mostra o consumo no relógio.

Limitação: só conta o uso feito naquele navegador. Para uso via API com chave, o `usageMetadata` de cada resposta traz os tokens consumidos.
