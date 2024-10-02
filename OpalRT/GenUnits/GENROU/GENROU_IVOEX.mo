within OpalRT.GenUnits.GENROU;
class GENROU_IVOEX
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // IVOEX Parameters
  parameter Integer IBUS_ex = IBUS "Located system bus" annotation(Dialog(tab = "IVOEX Parameters"));
  parameter String ID_ex = M_ID "Machine Identifier" annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real K1_ex = 1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real A1_ex = 0.5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real A2_ex = 0.5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real T1_ex = 0.05 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real T2_ex = 0.1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real MAX1_ex = 5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real MIN1_ex = -5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real K3_ex = 3 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real A3_ex = 0.5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real A4_ex = 0.5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real T3_ex = 0.1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real T4_ex = 0.1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real MAX3_ex = 5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real MIN3_ex = -5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real K5_ex = 1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real A5_ex = 0.5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real A6_ex = 0.5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real T5_ex = 0.1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real T6_ex = 0.1 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real MAX5_ex = 5 annotation(Dialog(tab = "IVOEX Parameters"));
  parameter Real MIN5_ex = -2 annotation(Dialog(tab = "IVOEX Parameters"));
  OpalRT.Electrical.Control.Excitation.IVOEX ivoex1(IBUS = IBUS_ex, ID = ID_ex, K1 = K1_ex, A1 = A1_ex, A2 = A2_ex, T1 = T1_ex, T2 = T2_ex, MAX1 = MAX1_ex, MIN1 = MIN1_ex, K3 = K3_ex, A3 = A3_ex, A4 = A4_ex, T3 = T3_ex, T4 = T4_ex, MAX3 = MAX3_ex, MIN3 = MIN3_ex, K5 = K5_ex, A5 = A5_ex, A6 = A6_ex, T5 = T5_ex, T6 = T6_ex, MAX5 = MAX5_ex, MIN5 = MIN5_ex) annotation(Placement(visible = true, transformation(origin={-10,20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {50, -2.88658e-15}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {36, 28}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-72,18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ivoex1.dVREF, dVREF) annotation(Line(points = {{-35, 5}, {-41.8692,
          5}, {-41.8692, -16.8224}, {-41.8692, -16.8224}}, color = {0, 0, 127}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{36,28},{50.1089,28},{
          50.1089,20},{50,20}}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{30,-16},{
          26.7782,-16},{26.7782,-12},{30,-12}}, color = {0,0,127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{50,-20},{50,-80.8926},{
          100,-80.8926},{100,-80}}));
  connect(ivoex1.EFD0, genrou1.EFD0)
    annotation (Line(points = {{15,0},{29.6,0}}, color={0,0,127}));
  connect(ivoex1.EFD, genrou1.EFD) annotation (Line(points = {{15,5},{22.5,5},{
          22.5,5.6},{30,5.6}}, color = {0,0,127}));
  connect(ivoex1.ETERM0, genrou1.ETERM0)
    annotation (Line(points = {{15,10},{29.6,10}}, color={0,0,127}));
  connect(ivoex1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{15,16.5},{
          22.5,16.5},{22.5,16},{29.6,16}}, color = {0,0,127}));
  connect(genrou1.VI, ivoex1.VI) annotation(Line(points = {{70, -2.88658e-15}, {91.2596, -2.88658e-15},
         {91.2596, 34.9614}, {17.2237, 34.9614}, {17.2237, 34.9614}}, color = {0, 0, 127}));
  connect(ivoex1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-35,32.5},
          {-44,32.5},{-44,54},{82,54},{82,10},{70,10}}, color = {0,0,127}));
  connect(const.y, ivoex1.VOEL)
    annotation (Line(points = {{-61,18},{-35,18}}, color={0,0,127}));
  connect(ivoex1.VOTHSG, ivoex1.VOEL) annotation (Line(points = {{-35,11},{-42,
          11},{-42,18},{-35,18}}, color = {0,0,127}));
  connect(ivoex1.VUEL, ivoex1.VOEL) annotation (Line(points = {{-35,25},{-42,25},
          {-42,18},{-35,18}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-21.8996, -6.14032}, extent = {{-50.07, 20.92}, {95.5372, -18.6885}}, textString = "GENROU_IVOEX"), Rectangle(origin = {0.13947, -0.13947}, extent = {{-99.4421, 99.1632}, {99.4421, -99.1632}}), Text(origin = {-7.7612, -76.2352}, extent = {{47.559, 13.3886}, {95.54, -18.69}}, textString = "PIN"), Text(origin = {-130.546, 57.5165}, extent = {{47.56, 13.39}, {78.72, -10.09}}, textString = "TRIP"), Text(origin = {-129.851, -42.374}, extent = {{47.56, 13.39}, {78.72, -10.09}}, textString = "dVREF")}));
end GENROU_IVOEX;
