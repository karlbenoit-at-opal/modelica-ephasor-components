within OpalRT.GenUnits.GENROE;
class GENROE_SCRX_GAST
  parameter Real partType = 1;
  // GENROE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROE Parameters"));
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  // GAST Parameters
  parameter Real R_tg = 0.01 "Speed droop" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T1_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T2_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T3_tg = 0.3 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real AT_tg = 0.12 "Ambient temperature load limit" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real KT_tg = 0.2 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMAX_tg = 0.12 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMIN_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real DTURB_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  //
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {0, 20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-88, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(IBUS = IBUS, ID = M_ID, TA_TB = TA_TB_ex, TB = TB_ex, K = K_ex, TE = TE_ex, EMIN = EMIN_ex, EMAX = EMAX_ex, CSWITCH = CSWITCH_ex, rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin = {-90, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROE genroe1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-5.625, -5.625}, {5.625, 5.625}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-140, 40}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
equation
  connect(dVREF, scrx1.dVREF) annotation(Line(points = {{-140, 40}, {-105.023, 40}, {-105.023, 41.7481}, {-99.8042, 41.7481}, {-99.8042, 41.7481}}));
  connect(dGREF, gast1.dGREF) annotation(Line(points = {{-120, 20}, {-111.328, 20}, {-111.328, 33.7029}, {-97.8472, 33.7029}, {-97.8472, 33.7029}}));
  connect(genroe1.VI, scrx1.VI) annotation(Line(points = {{-50, 40}, {-41.9422, 40}, {-41.9422, 54.0141}, {-79.2413, 54.0141}, {-79.2413, 54.0141}}, color = {0, 0, 127}));
  connect(genroe1.VI, gast1.VI) annotation(Line(points = {{-50, 40}, {-41.775, 40}, {-41.775, 6.4136}, {-109.551, 6.4136}, {-109.551, 25.8277}, {-98.1107, 25.8277}, {-98.1107, 25.8277}}, color = {0, 0, 127}));
  connect(genroe1.MBASE, gast1.MBASE) annotation(Line(points = {{-50, 34.6}, {-43.5085, 34.6}, {-43.5085, 8.32034}, {-107.471, 8.32034}, {-107.471, 21.8409}, {-98.4574, 21.8409}, {-98.4574, 21.8409}}, color = {0, 0, 127}));
  connect(genroe1.p, bus0) annotation(Line(points = {{-60, 30}, {-60, 20}, {0, 20}}));
  connect(TRIP, genroe1.TRIP) annotation(Line(points = {{-60, 80}, {-59.9198, 80}, {-59.9198, 50}, {-60, 50}}));
  connect(scrx1.EFD0, genroe1.EFD0) annotation(Line(points = {{-80, 40}, {-70.2, 40}}, color = {0, 0, 127}));
  connect(scrx1.EFD, genroe1.EFD) annotation(Line(points = {{-80, 42}, {-76, 42}, {-76, 42.8}, {-70, 42.8}}, color = {0, 0, 127}));
  connect(scrx1.ETERM0, genroe1.ETERM0) annotation(Line(points = {{-80, 44}, {-76, 44}, {-76, 45}, {-70.2, 45}}, color = {0, 0, 127}));
  connect(scrx1.EX_AUX, genroe1.EX_AUX) annotation(Line(points = {{-80, 46.6}, {-76, 46.6}, {-76, 48}, {-70.2, 48}}, color = {0, 0, 127}));
  connect(scrx1.XADIFD, genroe1.XADIFD) annotation(Line(points = {{-100, 53}, {-106, 53}, {-106, 64}, {-42, 64}, {-42, 45}, {-50, 45}}, color = {0, 0, 127}));
  connect(gast1.PMECH, genroe1.PMECH) annotation(Line(points = {{-78, 34}, {-70, 34}}, color = {0, 0, 127}));
  connect(genroe1.PMECH0, gast1.PMECH0) annotation(Line(points = {{-70, 32}, {-78, 32}}, color = {0, 0, 127}));
  connect(gast1.SLIP, genroe1.SLIP) annotation(Line(points = {{-98, 18}, {-106, 18}, {-106, 10}, {-46, 10}, {-46, 32}, {-50, 32}}, color = {0, 0, 127}));
  connect(const.y, scrx1.VUEL) annotation(Line(points = {{-109, 50}, {-100, 50}}, color = {0, 0, 127}));
  connect(scrx1.VOEL, scrx1.VUEL) annotation(Line(points = {{-100, 47.2}, {-104, 47.2}, {-104, 50}, {-100, 50}}, color = {0, 0, 127}));
  connect(scrx1.VOTHSG, scrx1.VUEL) annotation(Line(points = {{-100, 44.4}, {-104, 44.4}, {-104, 50}, {-100, 50}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-116.79, 25.76}, {128.56, -28.59}}, textString = "GENROE_SCRX_GAST2A"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}));
end GENROE_SCRX_GAST;
