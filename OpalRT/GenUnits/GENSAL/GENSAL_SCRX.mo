within OpalRT.GenUnits.GENSAL;
class GENSAL_SCRX
  parameter Real partType = 1;
  // GENSAL Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAL Parameters"));
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAL gensal1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-18, 10}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(IBUS = IBUS, ID = ID, TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, CSWITCH = CSWITCH_ex, rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin = {10, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-20, -6}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(scrx1.VI, gensal1.VI) annotation(Line(points = {{20, 14}, {57.8999, 14}, {57.8999, 0}, {50.5761, 0}, {50.5761, 0}}, color = {0, 0, 127}));
  connect(dVREF, scrx1.dVREF) annotation(Line(points = {{-20, -6}, {-11.0549, -6}, {-11.0549, 2.21098}, {-0.138186, 2.21098}, {-0.138186, 2.21098}}));
  connect(TRIP, gensal1.TRIP) annotation(Line(points = {{40, 35}, {40.1656, 35}, {40.1656, 10}, {40, 10}}));
  //
  connect(gensal1.p, bus0) annotation(Line(points = {{40, -10}, {76, -10}, {76, -10}, {80, -10}}));
  connect(scrx1.EFD0, gensal1.EFD0) annotation(Line(points = {{20, 0}, {29.8, 0}}, color = {0, 0, 127}));
  connect(scrx1.EFD, gensal1.EFD) annotation(Line(points = {{20, 2}, {26, 2}, {26, 2.8}, {30, 2.8}}, color = {0, 0, 127}));
  connect(scrx1.ETERM0, gensal1.ETERM0) annotation(Line(points = {{20, 4}, {26, 4}, {26, 5}, {29.8, 5}}, color = {0, 0, 127}));
  connect(scrx1.EX_AUX, gensal1.EX_AUX) annotation(Line(points = {{20, 6.6}, {26, 6.6}, {26, 8}, {29.8, 8}}, color = {0, 0, 127}));
  connect(scrx1.XADIFD, gensal1.XADIFD) annotation(Line(points = {{0, 13}, {-6, 13}, {-6, 22}, {54, 22}, {54, 5}, {50, 5}}, color = {0, 0, 127}));
  connect(const.y, scrx1.VUEL) annotation(Line(points = {{-12.5, 10}, {0, 10}}, color = {0, 0, 127}));
  connect(scrx1.VOEL, scrx1.VUEL) annotation(Line(points = {{0, 7.2}, {-6, 7.2}, {-6, 10}, {0, 10}}, color = {0, 0, 127}));
  connect(scrx1.VOTHSG, scrx1.VUEL) annotation(Line(points = {{0, 4.4}, {-6, 4.4}, {-6, 10}, {0, 10}}, color = {0, 0, 127}));
  connect(gensal1.PMECH0, gensal1.PMECH) annotation(Line(points = {{30, -8}, {28, -8}, {28, -6}, {30, -6}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-57.06, 28.58}, extent = {{143.62, -49.31}, {-24.72, 14.46}}, textString = "GENSAL_SCRX"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAL_SCRX;
