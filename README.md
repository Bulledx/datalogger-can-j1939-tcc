# Datalogger CAN/J1939 de baixo custo

Repositório do projeto de TCC **Datalogger de baixo custo para Controller Area Network**, desenvolvido para validação em bancada de mensagens CAN/J1939 com ESP32, MCP2515, SavvyCAN e cartão SD.

## Objetivo

O projeto implementa um datalogger CAN/J1939 de baixo custo capaz de registrar eventos diagnósticos em arquivos CSV, preservando dados antes, durante e após a ocorrência de uma falha ativa.

## Arquitetura resumida

Fluxo utilizado no ensaio de bancada:

```text
SavvyCAN -> CANable / USB2CAN -> MCP2515 + Transceiver -> ESP32 -> Cartão SD
```

O SavvyCAN gera mensagens J1939 simuladas, a interface USB2CAN transmite essas mensagens ao barramento CAN, o MCP2515 recebe os frames CAN e o ESP32 processa as mensagens EEC1 e DM1, salvando os eventos no cartão SD.

## Estrutura do repositório

```text
firmware/
  platformio.ini
  src/
    main.cpp

savvycan/
  Final_Scripting.js
```

## Firmware

O firmware do ESP32 realiza:

- inicialização do MCP2515;
- inicialização do cartão SD;
- sincronização de data e hora via NTP;
- leitura de mensagens CAN/J1939;
- decodificação de EEC1;
- identificação de DM1 ativo;
- gravação dos eventos em arquivos CSV.

No ensaio final, a janela de evento foi configurada para:

- 5 minutos de contexto anterior ao evento;
- 5 minutos de falha ativa simulada;
- 5 minutos de contexto posterior ao evento.

## Script SavvyCAN

O script SavvyCAN simula mensagens J1939 em bancada. Ele envia mensagens EEC1 em operação normal e, durante uma falha simulada, transmite mensagens DM1 com SPN e FMI associados ao evento.

## Ambiente de desenvolvimento

- Visual Studio Code
- PlatformIO
- ESP32 DevKit
- MCP2515 com transceiver CAN
- Cartão SD / módulo SD SPI
- SavvyCAN
- CANable / USB2CAN

## Observação

Este repositório foi disponibilizado para rastreabilidade do código-fonte utilizado no Trabalho de Conclusão de Curso. Versões futuras podem incluir ajustes, novos PGNs, novas regras de gatilho e melhorias na organização dos arquivos de saída.
