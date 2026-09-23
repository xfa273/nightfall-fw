# CAD data

KiCad原本は `kicad/CM_Nightfall-Air-2a/` にあります。

外部CADから受領したDXFは、KiCadプロジェクト内の `source/` にコピーしてあります。KiCad側では次のように変換しています。

- `Main-PCB.DXF`: `Edge.Cuts`
- `Main-PCB_Silk.DXF`: `F.Silkscreen`
- 直径1.5 mmの4穴: NPTH
- 中央の直径14 mm円: `Edge.Cuts` の基板切り欠き

