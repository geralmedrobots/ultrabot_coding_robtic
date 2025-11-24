# Plano de Validação e Prontidão Operacional

Este plano consolida as ações necessárias para levar o robô móvel de navegação hospitalar a um estado auditável em produção. Cada seção inclui objetivos, procedimentos, métricas/evidências e artefatos esperados.

## 1. Validação Hardware-in-the-Loop do `ethercat_driver`
- **Objetivo:** Validar controle de drives de produção com SOEM, confirmando watchdog, E-stop e limpeza de falhas.
- **Procedimento:**
  - Bring-up com drives reais em modo seguro (corrente/velocidade limitada).
  - Forçar cenários: ausência de `cmd`, E-stop físico, remoção/retorno de nó, erro de drive.
  - Capturar traces de osciloscópio ou diagnósticos dos drives durante cada cenário.
  - Registrar códigos de falha e tempos de recuperação.
- **Métricas/Evidências:**
  - Tempo até watchdog → stop seguro (ms).
  - Tempo de desacionamento/reaquisição após E-stop (s).
  - Log SOEM + diagnósticos de drive anexados.
- **Artefatos:** `docs/evidence/ethercat_watchdog_*.csv`, `docs/evidence/drive_scope_*.png`, checklist assinada pelo responsável de segurança.

## 2. Navegação e SLAM (Nav2)
- **Objetivo:** Finalizar tuning de árvores de comportamento, DWB e costmaps; concluir mapeamento/localização com SLAM Toolbox e AMCL.
- **Procedimento:**
  - Ajustar parâmetros do BT (árvore padrão + variantes de recuperação) e registrar resultados de trilhas.
  - Calibrar DWB (footprint, inflation, critic weights) com logs de bag em corredores e áreas abertas.
  - Gerar mapa com SLAM Toolbox (modo contínuo) e validar loop closures.
  - Trocar para AMCL com mapa congelado e validar re-localização após desvio.
- **Métricas/Evidências:**
  - Sucesso em 10/10 rotas hospitalar (dock → ala → dock) sem colisões.
  - `costmap` sem inflation leaks; footprints conferidos.
  - Mapas e `pose graph` exportados (`.posegraph`, `.yaml`, `.pgm`).
- **Artefatos:** `maps/hospital_map.yaml|pgm`, `maps/hospital_graph.posegraph`, relatório de tuning (`docs/evidence/nav2_tuning.md`).

## 3. Percepção e Fusão de Sensores
- **Objetivo:** Calibrar RealSense/Ouster, alimentar costmaps com nuvens de pontos e adicionar IMU ao pipeline de odometria.
- **Procedimento:**
  - Calibração extrínseca RealSense ↔ base e Ouster ↔ base com `apriltag` ou alvo planar; salvar `TF`.
  - Pipeline de nuvem de pontos → voxel/inflation → costmap 3D->2D; validar latência.
  - Fusão IMU no `ekf`/`ukf` de odometria; teste em corredores longos (redução de drift).
- **Métricas/Evidências:**
  - Erro de reprojeção < 1 px (RealSense) / 3 cm (Ouster).
  - Atualização de costmap < 100 ms em cenário típico.
  - Drift longitudinal reduzido ≥30% com IMU.
- **Artefatos:** `config/tf/calibration_realsense.yaml`, `config/tf/calibration_ouster.yaml`, `docs/evidence/imu_fusion_results.md`.

## 4. Diagnóstico e Observabilidade
- **Objetivo:** Expor saúde do sistema e criar painéis auditáveis.
- **Procedimento:**
  - Publicar métricas via `diagnostic_updater` em nós críticos (driver, supervisor, nav2 stack).
  - Encaminhar métricas/rosout para Prometheus + Grafana (ou similar) com alertas de watchdog, E-stop, perda de TF, latência de costmap.
  - Validar alertas automáticos com cenários induzidos.
- **Métricas/Evidências:**
  - Dashboards salvos (`.json`) e screenshots datados.
  - Alertas disparam em <5s para watchdog/E-stop.
- **Artefatos:** `docs/observability/grafana/*.json`, `docs/observability/screenshots/*.png`, playbook de alertas `docs/observability/alerts.md`.

## 5. Segurança e Compliance
- **Objetivo:** Substituir keystores de desenvolvimento, habilitar secure boot/criptografia e registrar resultados de pentest.
- **Procedimento:**
  - Gerar keystore de produção e distribuir via canal seguro; revogar chaves de desenvolvimento.
  - Habilitar secure boot e criptografia de disco no hardware alvo; registrar números de série e datas.
  - Executar pentest focado em DDS, OTA e portas expostas; documentar achados.
- **Métricas/Evidências:**
  - Checklist de troca de chaves (datas, responsáveis, hashes públicos).
  - Relatório de pentest com severidade/classificação e plano de mitigação.
- **Artefatos:** Atualização em `Navigation/SECURITY_SUMMARY.md` com hashes de keystore e resumo do pentest, laudos em `docs/security/pentest_*.pdf`.

## 6. CI e Testes Integrados
- **Objetivo:** Garantir execução automatizada de `colcon test` com SOEM e cenários de simulação (Gazebo/RMF).
- **Procedimento:**
  - Provisionar runner com ROS 2 + SOEM e permissões de rede para EtherCAT virtual/hardware.
  - Adicionar testes de lançamento cobrindo docking, evasão de obstáculos e agendamento de frotas (RMF) em Gazebo.
  - Incluir artefatos de registro (bags, logs) e thresholds de sucesso.
- **Métricas/Evidências:**
  - `colcon test` passando no runner dedicado.
  - Sucesso ≥95% nas missões de simulação (dock, corredor, interseção com pedestres simulados).
- **Artefatos:** Logs de CI (`ci/colcon_test.log`), pacotes de mundo/simulações `simulation/worlds/*.world`, `test/launch/*.test.py`.

## 7. Responsáveis e Assinaturas
- Definir responsáveis por domínio (drives, navegação, percepção, segurança, CI) e coletar assinaturas/data após cada bloco concluído.
- Reter evidências em repositório privado ou armazenamento interno com acesso de auditoria.

## 8. Entregáveis Finais
- Checklist completa assinada.
- Pasta `docs/evidence/` contendo logs, capturas e relatórios.
- Atualização de `Navigation/SECURITY_SUMMARY.md` com mudanças de keystore e resultados de pentest.
- Dashboards e alertas exportados para consulta de auditoria.
