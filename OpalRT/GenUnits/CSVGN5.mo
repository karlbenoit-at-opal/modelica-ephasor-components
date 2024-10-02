within OpalRT.GenUnits;
class CSVGN5
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.06 annotation(Dialog(tab = "General"));
  parameter Real P_gen = 0 annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 100 annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence (NOT USED)" annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real ZSOURCE_IM = 999 "Machine source impedence (NOT USED)" annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real TS1 = 0.01 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real VEMAX = 0.15 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real TS2 = 0.1 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real TS3 = 5 ">0" annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real TS4 = 0.01 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real TS5 = 0.01 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real KSVS = 400 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real KSD = 1 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real BMAX = 1 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real BpMAX = 1 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real BpMIN = -0.5 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real BMIN = -0.5 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real TS6 = 0.05 ">0" annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real DV = 0.15 annotation(Dialog(tab = "CSVGN5 Parameters"));
  parameter Real ICONM = 0 "Remote bus Number (NOT USED)" annotation(Dialog(tab = "ICONS"));
  input OpalRT.NonElectrical.Connector.InputInterfacePin VOTHSG annotation(Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-200, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {40, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.FACTS.CSVGN5 csvgn5(Vt_abs = Vt_abs, Vt_ang = Vt_ang, P_gen = P_gen, Q_gen = Q_gen, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, ZSOURCE_IM = ZSOURCE_IM, TS1 = TS1, VEMAX = VEMAX, TS2 = TS2, TS3 = TS3, TS4 = TS4, TS5 = TS5, KSVS = KSVS, KSD = KSD, BMAX = BMAX, BpMAX = BpMAX, BpMIN = BpMIN, BMIN = BMIN, TS6 = TS6, DV = DV) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-35, -35}, {35, 35}}, rotation = 0)));
equation
  connect(csvgn5.VOTHSG, VOTHSG) annotation(Line(points = {{-75, -24.5}, {-102.178, -24.5}, {-102.178, 57.4257}, {-102.178, 57.4257}}, color = {0, 0, 127}));
  connect(csvgn5.VREF0, csvgn5.VREF) annotation(Line(points = {{-75, 14}, {-77.6238, 14}, {-77.6238, 24.5545}, {-77.6238, 24.5545}}, color = {0, 0, 127}));
  connect(csvgn5.p, bus0) annotation(Line(points = {{-5, -28}, {38.0198, -28}, {38.0198, -40.7921}, {38.0198, -40.7921}}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-72.35, 23.36}, {72.35, -23.36}}, textString = "CSVGN5"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN"), Text(origin = {-148.022, 54.1842}, extent = {{-39.43, 24.94}, {47.9821, -11.5923}}, textString = "VOTHSG")}));
end CSVGN5;
