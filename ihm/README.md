# IHM FUT-ROBO

Console desktop local para operar a visao, a estrategia e a calibracao do projeto.

## Inicio rapido no Windows

Dentro da própria pasta `ihm`, dê dois cliques em:

```text
iniciar_ihm.bat
```

O inicializador usa a pasta em que ele está localizado e procura primeiro por
`py -3`, usando `python` como alternativa. Portanto, a pasta `ihm` pode ser
copiada para outro projeto sem alterar o inicializador.

## Início rápido no Linux

Coloque a pasta `ihm` na raiz do projeto, no mesmo nível de `vision`,
`strategy`, `firmware` e `common`. Depois, execute uma vez:

```bash
chmod +x ihm/iniciar_ihm.sh
```

Para abrir a IHM:

```bash
./ihm/iniciar_ihm.sh
```

No Ubuntu/Debian, a interface precisa do Tkinter:

```bash
sudo apt install python3 python3-tk
```

O script verifica automaticamente o Python, o Tkinter e a posição da pasta no
projeto. Para verificar sem abrir a janela:

```bash
./ihm/iniciar_ihm.sh --check
```

## Executar

Requer Python 3 com suporte a Tkinter. Nao existem pacotes Python externos.

No PowerShell, a partir da raiz do repositorio:

```powershell
python .\ihm\app.py
```

Se o comando instalado for `py`:

```powershell
py -3 .\ihm\app.py
```

## Recursos

- inicia e encerra os executaveis de visao e estrategia;
- configura o CMake na primeira vez e compila os modulos;
- mostra bola, robos e orientacoes recebidos no multicast UDP;
- mostra taxa de pacotes, total da sessao, idade do frame e tempo de operacao;
- desenha grade, coordenadas, gols, equipes e vetores de orientacao no campo;
- centraliza a saida dos processos em uma aba de logs;
- oferece navegacao lateral e atalhos `Ctrl+1`, `Ctrl+2` e `Ctrl+3`;
- permite copiar os eventos para a area de transferencia;
- permite editar `vision/config.yaml` sem dependencia de YAML para Python;
- inicia recalibracao do campo ou de uma cor, preservando o arquivo anterior;
- acompanha se os seis arquivos de calibracao estao prontos;
- envia tres comandos de parada ao firmware configurado.

Os cliques de calibracao ainda sao recebidos pela janela OpenCV da visao. A IHM cuida do backup, inicia o processo e explica a ordem dos cliques.

## Antes do primeiro uso

O CMake, o compilador C++23, o OpenCV e o Asio precisam estar instalados e
disponiveis no `PATH`. A propria IHM configura a pasta `build` quando necessario
e executa a compilacao; nao e preciso digitar os comandos manualmente.

## Testes

```powershell
python -m unittest discover -s .\ihm\tests -v
```

Os testes validam o formato binario compartilhado com C/C++ e a leitura/escrita conservadora do arquivo OpenCV YAML.
