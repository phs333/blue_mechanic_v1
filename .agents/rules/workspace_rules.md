# Diretrizes do Projeto Blue Mechanic V1

## Execução Autônoma e Direta
- **Modo Direto**: Não entre em Planning Mode nem crie planos de implementação intermediários exigindo aprovação/confirmação para tarefas de código, correções, testes ou melhorias. Execute as alterações diretamente no código.
- **Validação Automática**: Ao realizar mudanças no código, rode imediatamente os testes relevantes (`.\.venv\Scripts\python.exe -m unittest ...`) e reporte o resultado final diretamente.
- **Sem Interrupções**: Só solicite confirmação ou pergunte ao usuário se houver risco real de perda de dados irreversível ou ambiguidade arquitetural insuperável.

## Nomenclatura e Terminologia
- **Termo "Nodes" / "Nós"**: Refira-se aos 10 dispositivos ESP32-S3 conectados via CAN exclusivamente como **"nodes"** ou **"nós"** (ex: "Node 1..10", "Painel dos 10 Nós", "Status dos Nós").
- **Nunca use o termo "frota" ou "fleet"**.
