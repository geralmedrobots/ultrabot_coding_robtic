# Remaining Work Checklist for Ultrabot

This checklist consolidates the outstanding work required to reach full operational readiness.
It draws from the operational readiness snapshot, roadmap milestones, and safety guidance so
contributors can quickly identify the next actionable tasks.

## Resumo rápido do que ainda falta (PT-BR)

- **Validação em hardware real:** Exercitar o `ethercat_driver` com drives físicos e
  registrar evidências de watchdog e E-stop para comprovar conformidade de segurança.
- **Navegação e SLAM:** Concluir ajustes de Nav2 (árvores de comportamento, DWB,
  costmaps) e finalizar mapeamento/localização com SLAM Toolbox e AMCL.
- **Percepção e fusão de sensores:** Calibrar RealSense/Ouster, alimentar costmaps com
  nuvens de pontos e adicionar IMU ao pipeline de odometria para robustez em corredores.
- **Diagnóstico e observabilidade:** Publicar métricas via `diagnostic_updater`, criar
  alertas automáticos e dashboards (Grafana ou similar) para auditoria hospitalar.
- **Segurança e compliance:** Substituir keystores de desenvolvimento por segredos de
  produção, habilitar secure boot/criptografia e registrar resultados de pentests em
  `Navigation/SECURITY_SUMMARY.md`.
- **CI e testes integrados:** Disponibilizar runner com ROS 2 + SOEM para rodar
  `colcon test`, acrescentar testes de lançamento e cenários Gazebo/RMF que cubram
  docking, evasão de obstáculos e agendamento de frotas.

## Hardware-in-the-Loop Validation

- [ ] **(PT-BR)** Validar o `ethercat_driver` com drives de produção usando SOEM, confirmando
      timeouts do watchdog, limpeza de falhas e tratamento de E-stop em hardware real.
- [ ] **(PT-BR)** Capturar traces de osciloscópio ou diagnósticos dos drives durante o bring-up
      para documentar conformidade com requisitos de segurança hospitalar.
- [ ] **(PT-BR)** Exercitar gerenciamento de bateria, carregamento e docking com o supervisor de
      segurança rodando em paralelo à stack de navegação.

- [ ] Validate the `ethercat_driver` against production drives with SOEM installed, confirming
      watchdog timeouts, fault clearing, and emergency-stop handling on physical hardware.
- [ ] Capture oscilloscope traces or drive diagnostics during bring-up to document compliance with
      hospital safety requirements.
- [ ] Exercise battery management, charging, and docking workflows with the safety supervisor
      running alongside the navigation stack.

## Autonomy Stack Completion

- [ ] **(PT-BR)** Configurar árvores de comportamento e controladores do Nav2 usando datasets do
      hospital e documentar parâmetros em `docs/ROADMAP.md`.
- [ ] **(PT-BR)** Integrar pipelines do SLAM Toolbox e AMCL para gerar/consumir mapas da
      instalação, incluindo corridas de validação de acurácia de localização.
- [ ] **(PT-BR)** Fundir odometria das rodas com IMU via `robot_localization` para melhorar pose
      em corredores longos e elevadores.

- [ ] Configure Nav2 behavior trees and controllers using datasets collected from the hospital
      environment; document tuning parameters in `docs/ROADMAP.md`.
- [ ] Integrate SLAM Toolbox and AMCL pipelines to generate and consume facility maps, including
      validation runs that confirm localization accuracy.
- [ ] Fuse wheel odometry with IMU data through `robot_localization` to improve pose estimates in
      long corridors and elevators.

## Perception & Sensor Integration

- [ ] **(PT-BR)** Capturar intrínsecos/extrínsecos da Intel RealSense D455 por robô e salvar as
      calibrações em overrides `config/realsense_d455.yaml`.
- [ ] **(PT-BR)** Exportar metadados Ouster para cada LiDAR (`os_config.json`) e validar fidelidade
      das nuvens de pontos em corredores hospitalares, atualizando `config/ouster_lidar.yaml`.
- [ ] **(PT-BR)** Encaminhar tópicos de profundidade e nuvem de pontos para os costmaps e camadas
      de obstáculos do Nav2, documentando a configuração no guia de bring-up de navegação.

- [ ] Capture Intel RealSense D455 intrinsics/extrinsics per robot and store the calibration in
      `config/realsense_d455.yaml` overrides.
- [ ] Export Ouster metadata for each LiDAR (`os_config.json`) and validate point-cloud fidelity
      against hospital corridors, updating `config/ouster_lidar.yaml` as required.
- [ ] Wire depth and point-cloud topics into Nav2 costmaps and obstacle layers, documenting the
      configuration in the navigation bring-up guide.

## Diagnostics and Observability

- [ ] **(PT-BR)** Conectar diagnósticos ROS 2 (`diagnostic_updater`, `system_metrics_collector`) para
      publicar métricas de CPU, memória, rede e saúde dos drives no dashboard da frota.
- [ ] **(PT-BR)** Adicionar alertas automáticos para kicks do watchdog, transições de falha e
      eventos de saturação de comandos.
- [ ] **(PT-BR)** Produzir dashboards Grafana (ou equivalente) para que a equipe do hospital possa
      auditar a saúde do sistema em tempo real.

- [ ] Wire ROS 2 diagnostics (`diagnostic_updater`, `system_metrics_collector`) to publish CPU,
      memory, network, and drive-health metrics for the fleet dashboard.
- [ ] Add automated alerts for watchdog kicks, fault transitions, and command saturation events.
- [ ] Produce Grafana dashboards or equivalent visualizations so hospital staff can audit system
      health in real time.

## Security Hardening

- [ ] **(PT-BR)** Substituir keystores SROS2 de desenvolvimento por segredos de produção e reforçar
      a geração de políticas no pipeline de deploy.
- [ ] **(PT-BR)** Habilitar secure boot e criptografia de disco na unidade de computação,
      documentando o processo de flash e gestão de chaves.
- [ ] **(PT-BR)** Executar pentests focados em interfaces do middleware ROS 2 e registrar ações de
      remediação em `Navigation/SECURITY_SUMMARY.md`.

- [ ] Replace development SROS2 keystores with production secrets and enforce policy generation as
      part of the deployment pipeline.
- [ ] Enable secure boot and disk encryption on the compute unit, documenting the flashing and key
      management process.
- [ ] Perform penetration testing focused on ROS 2 middleware interfaces, recording remediation
      actions in `Navigation/SECURITY_SUMMARY.md`.

## Continuous Integration & Testing

- [ ] **(PT-BR)** Provisionar um runner de CI com ROS 2 Humble/Jazzy e SOEM para executar
      `colcon test` em cada merge request.
- [ ] **(PT-BR)** Estender a suíte de unit tests com testes de integração baseados em launch usando
      `ros2/ros_testing` para cobrir transições de lifecycle e publicação de TF.
- [ ] **(PT-BR)** Simular cenários hospitalares no Gazebo (gz-sim) para validar docking, evasão de
      obstáculos e agendamento de frotas com adaptadores RMF.

- [ ] Provision a CI runner with ROS 2 Humble/Jazzy and SOEM so that `colcon test` can execute on
      each merge request.
- [ ] Extend the existing unit-test suite with launch-based integration tests using
      `ros2/ros_testing` to cover lifecycle transitions and TF publication.
- [ ] Simulate hospital scenarios in Gazebo (gz-sim) to validate docking, obstacle avoidance, and
      fleet scheduling with RMF adapters.

Progress updates should be recorded in the roadmap and operational readiness snapshot to maintain a
single source of truth for stakeholders.
