within OpalRT.GenUnits.GENSAE;
class GENSAE_IEEEX1_IEEEST_HYGOV
  parameter Real partType = 1;
  // GENSAE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Tdo_p = 10 "d-axis transient time constant" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Tdo_s = 0.05 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENSAE Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENSAE Parameters"));
  // IEEEX1 Parameters
  parameter Real TR_ex = 0.025 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real KA_ex = 98 annotation(Dialog(tab = "IEEEX1"));
  parameter Real TA_ex = 0.2 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TB_ex = 0.5 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TC_ex = 1 "(sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real VRMAX_ex = 9 "or zero" annotation(Dialog(tab = "IEEEX1"));
  parameter Real VRMIN_ex = -5 annotation(Dialog(tab = "IEEEX1"));
  parameter Real KE_ex = 0.5 "or zero" annotation(Dialog(tab = "IEEEX1"));
  parameter Real TE_ex = 0.35 "(>0) (sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real KF_ex = 0.03 annotation(Dialog(tab = "IEEEX1"));
  parameter Real TF1_ex = 0.4 "(>0) (sec)" annotation(Dialog(tab = "IEEEX1"));
  parameter Real Switch_ex = 0 annotation(Dialog(tab = "IEEEX1"));
  parameter Real E1_ex = 4 annotation(Dialog(tab = "IEEEX1"));
  parameter Real SE_E1_ex = 0.4 annotation(Dialog(tab = "IEEEX1"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "IEEEX1"));
  parameter Real SE_E2_ex = 0.5 annotation(Dialog(tab = "IEEEX1"));
  // HYGOV Parameters
  parameter Real R_tg = 0.06 "Permanent Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real r_tg = 0.4 "Temporary Droop" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tr_tg = 8 "(>0) Governor time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tf_tg = 0.05 "(>0) Filter time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Tg_tg = 0.2 "(>0) Servo time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real VELM_tg = 0.01 "Gate velocity limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMAX_tg = 0.601 "Maximum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real GMIN_tg = 0 "Minimum gate limit" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real TW_tg = 1.2 "(>0) Water time constant" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real At_tg = 2.5 "Trubine gain" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real Dturb_tg = 0 "Turbine damping" annotation(Dialog(tab = "HYGOV Parameters"));
  parameter Real qNL_tg = 0.5 "No power flow" annotation(Dialog(tab = "HYGOV Parameters"));
  //
  // IEEEST Parameters
  parameter Real A1_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T1_pss = 0.03 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T2_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T3_pss = 0.02 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T4_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T5_pss = 0.2 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T6_pss = 0.1 "(>0)(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real KS_pss = -5 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMAX_pss = 6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMIN_pss = -6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCU_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCL_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  // IEEEST ICONs
  parameter Real M0_pss = 1 "Stabilizer input code" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  parameter Real M1_pss = 1 "IB, remote bus number" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.TurbineGovernor.HYGOV hygov1(R = R_tg, r = r_tg, Tr = Tr_tg, Tf = Tf_tg, Tg = Tg_tg, VELM = VELM_tg, GMAX = GMAX_tg, GMIN = GMIN_tg, TW = TW_tg, At = At_tg, Dturb = Dturb_tg, qNL = qNL_tg, IBUS = IBUS, ID = ID) annotation(Placement(visible = true, transformation(origin={12,-14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -10}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin={-38,-14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.IEEEX1 ieeex11(TR = TR_ex, KA = KA_ex, TA = TA_ex, TB = TB_ex, TC = TC_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF1 = TF1_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={12,8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENSAE gensae1(IBUS = IBUS, ID = ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {40, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {45, 35}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(A1 = A1_pss, A2 = A2_pss, A3 = A3_pss, A4 = A4_pss, A5 = A5_pss, A6 = A6_pss, T1 = T1_pss, T2 = T2_pss, T3 = T3_pss, T4 = T4_pss, T5 = T5_pss, T6 = T6_pss, KS = KS_pss, LSMAX = LSMAX_pss, LSMIN = LSMIN_pss, VCU = VCU_pss, VCL = VCL_pss, M0 = M0_pss, M1 = M1_pss) annotation(Placement(visible = true, transformation(origin={-26,10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-22,30}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(gensae1.p, bus0) annotation(Line(points = {{40,-10},{75.3623,-10},{
          75.3623,-10},{80,-10}}));
  connect(TRIP, gensae1.TRIP) annotation(Line(points = {{40,35},{39.9586,35},{
          39.9586,10},{40,10}}));
  //
  connect(ieeex11.EFD0, gensae1.EFD0)
    annotation (Line(points = {{22,0},{29.8,0}}, color={0,0,127}));
  connect(ieeex11.EFD, gensae1.EFD) annotation (Line(points = {{22,2},{26,2},{
          26,2.8},{30,2.8}}, color = {0,0,127}));
  connect(ieeex11.ETERM0, gensae1.ETERM0) annotation (Line(points = {{22,4},{26,
          4},{26,5},{29.8,5}}, color = {0,0,127}));
  connect(ieeex11.EX_AUX, gensae1.EX_AUX) annotation (Line(points = {{22,6.6},{
          26,6.6},{26,8},{29.8,8}}, color = {0,0,127}));
  connect(gensae1.VI, ieeex11.VI) annotation(Line(points = {{50, 0}, {61.5563, 0},
         {61.5563, 14.1492}, {22.7554, 14.1492}, {22.7554, 14.1492}}, color = {0, 0, 127}));
  connect(ieeex11.XADIFD, gensae1.XADIFD) annotation (Line(points = {{2,13},{-4,
          13},{-4,24},{58,24},{58,5},{50,5}}, color = {0,0,127}));
  connect(hygov1.PMECH, gensae1.PMECH) annotation (Line(points = {{22,-6},{24,
          -6},{24,-6},{30,-6}}, color = {0,0,127}));
  connect(gensae1.PMECH0, hygov1.PMECH0) annotation (Line(points = {{30,-8},{26,
          -8},{26,-28},{22,-28},{22,-8}}, color = {0,0,127}));
  connect(hygov1.dGREF, dGREF) annotation(Line(points = {{2, -6}, {-34.6783, -6},
          {-34.6783, -7.85169}, {-34.6783, -7.85169}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, ieeex11.VOTHSG) annotation (Line(points = {{-16,4},{
          -8,4},{-8,4.4},{2,4.4}}, color = {0,0,127}));
  connect(ieeex11.dVREF, dVREF) annotation(Line(points = {{2, 2}, {-11.5594, 2},
          {-11.5594, -2.39913}, {-79.1712, -2.39913}, {-79.1712, -2.39913}}, color = {0, 0, 127}));
  connect(const.y, ieeex11.VOEL) annotation (Line(points = {{-16.5,30},{-14,30},
          {-14,7.2},{2,7.2}}, color = {0,0,127}));
  connect(ieeex11.VUEL, ieeex11.VOEL) annotation (Line(points = {{2,10},{-14,10},
          {-14,7.2},{2,7.2}}, color = {0,0,127}));
  connect(gensae1.SLIP, ieeest1.PSS_AUX[1]) annotation(Line(points = {{50, -8},
         {61.4105, -8}, {61.4105, -36.7588}, {-52.5125, -36.7588}, {-52.5125, 10.0649},
         {-36.3212, 10.0649}, {-36.3212, 10.0649}}, color = {0, 0, 127}));
  connect(gensae1.AccPower, ieeest1.PSS_AUX[2]) annotation(Line(points = {{50, -2.6},
         {61.4105, -2.6}, {61.4105, -36.7588}, {-52.5125, -36.7588}, {-52.5125, 10.0649},
         {-36.467, 10.0649}, {-36.467, 10.0649}}, color = {0, 0, 127}));
  connect(ieeest1.VI, ieeest1.VI2) annotation(Line(points = {{-36, 12}, {-42.01, 12},
         {-42.01, 6.12646}, {-36.1753, 6.12646}, {-36.1753, 6.12646}}, color = {0, 0, 127}));
  connect(gensae1.VI, ieeest1.VI) annotation(Line(points = {{50, 0}, {61.5563, 0},
         {61.5563, 41.7183}, {-42.01, 41.7183}, {-42.01, 11.9612}, {-36.7588, 11.9612},
         {-36.7588, 11.9612}}, color = {0, 0, 127}));
  connect(hygov1.SLIP, gensae1.SLIP) annotation (Line(points = {{2,-22},{0,-22},
          {0,-30},{54,-30},{54,-8},{50,-8}},color = {0,0,127}));
  connect(gensae1.VI, hygov1.VI) annotation(Line(points = {{50, 0}, {59.6309, 0},
          {59.6309, -34.6349}, {-2.77733, -34.6349}, {-2.77733, -14.05}, {1.96047,
          -14.05}, {1.96047, -14.05}}, color = {0, 0, 127}));
  connect(gensae1.MBASE, hygov1.MBASE) annotation(Line(points = {{50, -5.4},
          {57.6704, -5.4}, {57.6704, -33.1646}, {-1.63372, -33.1646}, {-1.63372,
          -18.1343}, {2.12384, -18.1343}, {2.12384, -18.1343}}, color = {0, 0, 127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation (Line(points = {{-35.8,
          4},{-40,4},{-40,10},{-36,10}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Documentation(info = "<html>

<p>
for The Stabilizer:
</p>

<ul>
<li> 1-M0 is currently accepts values 1,3 or 5;</li>
<li> 2-reading values of remote bus is currently disabled and only terminal values are read.(M0 = 0)</li>
</ul>

</html>"),
         Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {0.341686, -0.113895}, extent = {{-99.4305, 99.4305}, {99.4305, -99.4305}}), Text(origin = {-57.06, 28.58}, extent = {{143.62, -49.31}, {-24.72, 14.46}}, textString = "GENSAE_IEEEX1_IEEEST_HYGOV"), Text(origin = {67.6574, -76.3052}, extent = {{-17.77, 11.62}, {17.77, -11.62}}, textString = "PIN")}));
end GENSAE_IEEEX1_IEEEST_HYGOV;
