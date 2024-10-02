within OpalRT.GenUnits.GENROU;
class GENROU_IEEET1_IEEEG2_UEL1
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // IEEET1 Parameters
  // This Model is located at system bus IBUS
  parameter Real TR_ex = 0.025 "(sec)";
  parameter Real KA_ex = 98;
  parameter Real TA_ex = 0.2 "(sec)";
  parameter Real VRMAX_ex = 9 "or zero";
  parameter Real VRMIN_ex = -5;
  parameter Real KE_ex = 0.5 "or zero";
  parameter Real TE_ex = 0.35 "(>0) (sec)";
  parameter Real KF_ex = 0.03;
  parameter Real TF_ex = 0.4 "(>0) (sec)";
  parameter Real Switch_ex = 0;
  parameter Real E1_ex = 4;
  parameter Real SE_E1_ex = 0.4;
  parameter Real E2_ex = 5;
  parameter Real SE_E2_ex = 0.5;
  //
  // UEL1 Parameters
  parameter Real KUR_uel = 2 "UEL radius setting (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUC_uel = 1 "(pu) UEL center setting (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUF_uel = 1 "(pu) UEL excitation system stabilizergain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VURMAX_uel = 1 "UEL maximum limit for radius phasor magnitude (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUCMAX_uel = 1 "UEL maximum limit for operating point phasor magnitude (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUI_uel = 1 "UEL integral gain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUL_uel = 1 "UEL proportional gain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUIMAX_uel = 1 "UEL integrator output maximum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUIMIN_uel = 1 "UEL integrator output minimum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU1_uel = 1 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU2_uel = 1 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU3_uel = 1 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU4_uel = 1 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VULMAX_uel = 1 "UEL output maximum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VULMIN_uel = 1 "UEL output minimum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  //ieeeg2
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "IEEEG2"));
  parameter Real K_tg = 20 "K" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T1_tg = 20 "T1" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T2_tg = 20 "T2" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T3_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMAX_tg = 20 "PMAX (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMIN_tg = 20 "PMIN (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T4_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  OpalRT.Electrical.Control.Excitation.IEEET1 ieeet11(TR = TR_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KE = KE_ex, TE = TE_ex, KF = KF_ex, TF = TF_ex, Switch = Switch_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-8,10}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(
    ID = M_ID,
    P_gen = P_gen,
    Q_gen = Q_gen,
    Vt_abs = Vt_abs,
    Vt_ang = Vt_ang,
    SB = SB,
    fn = fn,
    ZSOURCE_RE = ZSOURCE_RE,
    Tdo_p = Tdo_p,
    Tdo_s = Tdo_s,
    Tqo_p = Tqo_p,
    Tqo_s = Tqo_s,
    H = H,
    D = D,
    Xd = Xd,
    Xq = Xq,
    Xd_p = Xd_p,
    Xq_p = Xq_p,
    Xd_s = Xd_s,
    Xl = Xl,
    S1 = S1,
    S12 = S12) annotation(Placement(visible = true, transformation(origin={56,-10}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-50,8}, extent={{-4,-4},
            {4,4}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={79,-85}, extent={{-7,-7}, {7,7}}, rotation = 0), iconTransformation(origin={100,-40},extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL1 uel11(
    KUR = KUR_uel,
    KUC = KUC_uel,
    KUF = KUF_uel,
    VURMAX = VURMAX_uel,
    VUCMAX = VUCMAX_uel,
    KUI = KUI_uel,
    KUL = KUL_uel,
    VUIMAX = VUIMAX_uel,
    VUIMIN = VUIMIN_uel,
    TU1 = TU1_uel,
    TU2 = TU2_uel,
    TU3 = TU3_uel,
    TU4 = TU4_uel,
    VULMAX = VULMAX_uel,
    VULMIN = VULMIN_uel) annotation(Placement(visible = true, transformation(origin={-80,10}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-102,-16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG2 ieeeg21(
    K = K_tg,
    T1 = T1_tg,
    T2 = T2_tg,
    T3 = T3_tg,
    PMAX = PMAX_tg,
    PMIN = PMIN_tg,
    T4 = T4_tg) annotation(Placement(visible = true, transformation(origin={-4,-38}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={43,29}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-100, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin={-42,-26}, extent={{-6,-6}, {6,6}}, rotation = 0), iconTransformation(origin={-100,-60},   extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation 
  connect(bus0,genrou1. p) annotation(Line(points={{79,-85},{56,-85},{56,-35}}));
  connect(ieeeg21.PMECH, genrou1.PMECH) annotation (Line(points = {{11,-26},{16,
          -26},{16,-25},{31,-25}}, color = {0,0,127}));
  connect(genrou1.PMECH0, ieeeg21.PMECH0) annotation (Line(points={{31,-30},{18,
          -30},{18,-29},{11,-29}}, color = {0,0,127}));
  connect(genrou1.SLIP, ieeeg21.SLIP) annotation (Line(points={{81,-30},{84,-30},
          {84,-56},{-24,-56},{-24,-50},{-19,-50}}, color = {0,0,127}));
  connect(uel11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-94.7,22},{
          -98,22},{-98,56},{26,56},{26,10},{30.5,10}}, color = {0,0,127}));
  connect(TRIP, genrou1.TRIP)
    annotation (Line(points = {{43,29},{56,29},{56,15}}, color={0,0,0}));
  connect(ieeet11.EFD0, genrou1.EFD0)
    annotation (Line(points = {{17,-10},{30.5,-10}}, color={0,0,127}));
  connect(ieeet11.EFD, genrou1.EFD) annotation (Line(points = {{17,-5},{24.5,-5},
          {24.5,-3},{31,-3}}, color = {0,0,127}));
  connect(ieeet11.ETERM0, genrou1.ETERM0) annotation (Line(points = {{17,0},{24,
          0},{24,2.5},{30.5,2.5}}, color = {0,0,127}));
  connect(ieeet11.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{17,6.5},{
          23.5,6.5},{23.5,10},{30.5,10}}, color = {0,0,127}));
  connect(uel11.VUEL, ieeet11.VUEL) annotation (Line(points = {{-64.4,10},{-50,
          10},{-50,15},{-33,15}},
                              color = {0,0,127}));
  connect(const.y, ieeet11.VOEL)
    annotation (Line(points = {{-45.6,8},{-33,8}}, color={0,0,127}));
  connect(ieeet11.VOTHSG, ieeet11.VOEL) annotation (Line(points = {{-33,1},{-40,
          1},{-40,8},{-33,8}}, color = {0,0,127}));
  connect(ieeet11.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-33,22.5},
          {-42,22.5},{-42,42},{90,42},{90,2.5},{81,2.5}}, color = {0,0,127}));
  connect(uel11.VF, ieeet11.VF) annotation (Line(points={{-94.7,10},{-104,10},{-104,
          52},{24,52},{24,30},{18,30}}, color = {0,0,127}));
  connect(dVREF, ieeet11.dVREF) annotation (Line(points={{-102,-16},{-40,-16},{-40,
          -5},{-33,-5}}, color={0,0,0}));
  connect(genrou1.VI, ieeet11.VI) annotation (Line(points={{81,-10},{94,-10},{94,
          46},{30,46},{30,25},{17,25}}, color={0,0,127}));
  connect(genrou1.VI, uel11.VI) annotation (Line(points={{81,-10},{94,-10},{94,46},
          {-102,46},{-102,16},{-94.7,16}}, color={0,0,127}));
  connect(genrou1.MBASE, ieeeg21.MBASE) annotation (Line(points={{81,-23.5},{88,
          -23.5},{88,-60},{-28,-60},{-28,-44},{-19,-44}}, color={0,0,127}));
  connect(genrou1.VI, ieeeg21.VI) annotation (Line(points={{81,-10},{94,-10},{94,
          -64},{-32,-64},{-32,-38},{-19,-38}}, color={0,0,127}));
  connect(dGREF, ieeeg21.dGREF) annotation (Line(points={{-42,-26},{-19,-26}}, color={0,0,0}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-81.55, 7.17}, extent = {{-12.53, 6.49}, {175.86, -14.01}}, textString = "GENROU_ESAC5A_IEEEG2_UEL1"), Rectangle(origin = {0.113895, -0.341686}, extent = {{-99.8861, 99.8861}, {99.8861, -99.8861}})}));
end GENROU_IEEET1_IEEEG2_UEL1;
