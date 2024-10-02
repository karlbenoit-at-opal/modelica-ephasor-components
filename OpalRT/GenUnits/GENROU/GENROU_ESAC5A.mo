within OpalRT.GenUnits.GENROU;
class GENROU_ESAC5A
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1000 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 100 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 0.95 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -2 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1200 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 60 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 10.2 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.5 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 1.02 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 8.2 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 3 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.5 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.5231 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.361 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.41 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.2 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.5 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.6 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // ESAC5A Parameters
  parameter Real TR_ex = 0.02 "(sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real KA_ex = 100 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TA_ex = 0.5 "(sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real VRMIN_ex = -5 "V" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TE_ex = 0.08 "(sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real KF_ex = 0.2 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TF1_ex = 1.2 "(>0) (sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TF2_ex = 2 "(>0) (sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real TF3_ex = 2 "(>0) (sec)" annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "ESAC5A Parameters"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "ESAC5A Parameters"));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {42, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, 2}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.ESAC5A esac5a1(TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, TF2 = TF2_ex, TF3 = TF3_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-6,34}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-48,50}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-84,30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {42, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(esac5a1.dVREF, dVREF) annotation(Line(points = {{-22, 24.4}, {-77.2504,
          24.4}, {-77.2504, 24.8773}, {-77.2504, 24.8773}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{22,4},{
          18.2062,4},{18.2062,8},{22,8}}, color = {0,0,127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{42,0},{76.8139,0},{
          76.8139,2},{80,2}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{42,60},{41.7664,60},{
          41.7664,40},{42,40}}));
  connect(esac5a1.EFD0, genrou1.EFD0) annotation (Line(points = {{10,21.2},{16,
          21.2},{16,20},{21.6,20}}, color = {0,0,127}));
  connect(esac5a1.EFD, genrou1.EFD) annotation (Line(points = {{10,24.4},{16,
          24.4},{16,25.6},{22,25.6}}, color = {0,0,127}));
  connect(esac5a1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{10,27.6},
          {18,27.6},{18,30},{21.6,30}}, color = {0,0,127}));
  connect(esac5a1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{10,31.76},
          {16,31.76},{16,36},{21.6,36}}, color = {0,0,127}));
  connect(genrou1.VI, esac5a1.VI) annotation(Line(points = {{62, 20}, {74.8072, 20},
          {74.8072, 43.4447}, {11.054, 43.4447}, {11.054, 43.4447}}, color = {0, 0, 127}));
  connect(esac5a1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-22,42},{
          -28,42},{-28,54},{70,54},{70,30},{62,30}}, color = {0,0,127}));
  connect(const.y, esac5a1.VOTHSG) annotation (Line(points = {{-40.3,50},{-30,
          50},{-30,28.24},{-22,28.24}}, color = {0,0,127}));
  connect(esac5a1.VOEL, esac5a1.VOTHSG) annotation (Line(points = {{-22,32.72},
          {-30,32.72},{-30,28.24},{-22,28.24}}, color = {0,0,127}));
  connect(esac5a1.VUEL, esac5a1.VOTHSG) annotation (Line(points = {{-22,37.2},{
          -26,37.2},{-26,38},{-30,38},{-30,28.24},{-22,28.24}}, color = {0,0,
          127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-1.02506, 0.113895}, extent = {{-96.0137, 95.5581}, {96.0137, -95.5581}}), Text(origin = {-35.42, 15.27}, extent = {{-48.63, 23.46}, {119.7, -34.17}}, textString = "GENROU_ESAC5A"), Text(origin = {-68.41, 59.17}, extent = {{-15.13, 7.5}, {15.13, -7.5}}, textString = "TRIP"), Text(origin = {-73.5783, -53.6271}, extent = {{-15.13, 7.5}, {26.375, -15.7999}}, textString = "dVREF")}));
end GENROU_ESAC5A;
