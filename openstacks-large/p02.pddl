(define (problem os-sequencedstrips-p170_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) 
(stacks-avail n0)

(waiting o1)
(includes o1 p106)

(waiting o2)
(includes o2 p62)(includes o2 p87)

(waiting o3)
(includes o3 p96)(includes o3 p151)(includes o3 p155)(includes o3 p158)

(waiting o4)
(includes o4 p5)(includes o4 p39)(includes o4 p44)(includes o4 p84)(includes o4 p95)(includes o4 p105)(includes o4 p111)(includes o4 p112)(includes o4 p125)(includes o4 p158)(includes o4 p167)

(waiting o5)
(includes o5 p105)(includes o5 p125)(includes o5 p135)(includes o5 p146)(includes o5 p163)(includes o5 p165)

(waiting o6)
(includes o6 p2)(includes o6 p3)(includes o6 p30)(includes o6 p35)(includes o6 p41)(includes o6 p71)(includes o6 p104)(includes o6 p138)(includes o6 p167)

(waiting o7)
(includes o7 p25)(includes o7 p26)(includes o7 p52)(includes o7 p87)(includes o7 p128)(includes o7 p137)(includes o7 p143)(includes o7 p144)

(waiting o8)
(includes o8 p42)(includes o8 p67)(includes o8 p69)(includes o8 p71)(includes o8 p142)

(waiting o9)
(includes o9 p93)(includes o9 p152)

(waiting o10)
(includes o10 p40)(includes o10 p50)(includes o10 p80)(includes o10 p116)(includes o10 p123)(includes o10 p136)(includes o10 p145)

(waiting o11)
(includes o11 p8)(includes o11 p34)(includes o11 p68)(includes o11 p82)(includes o11 p106)(includes o11 p119)(includes o11 p122)(includes o11 p124)(includes o11 p135)(includes o11 p138)

(waiting o12)
(includes o12 p36)

(waiting o13)
(includes o13 p12)(includes o13 p44)(includes o13 p75)(includes o13 p129)(includes o13 p161)

(waiting o14)
(includes o14 p8)(includes o14 p42)(includes o14 p48)(includes o14 p127)(includes o14 p154)

(waiting o15)
(includes o15 p49)(includes o15 p50)(includes o15 p106)

(waiting o16)
(includes o16 p18)(includes o16 p23)(includes o16 p90)(includes o16 p91)(includes o16 p161)(includes o16 p163)

(waiting o17)
(includes o17 p11)(includes o17 p42)(includes o17 p52)(includes o17 p103)(includes o17 p106)(includes o17 p108)(includes o17 p123)(includes o17 p124)(includes o17 p155)

(waiting o18)
(includes o18 p29)(includes o18 p33)(includes o18 p35)(includes o18 p37)(includes o18 p64)(includes o18 p107)

(waiting o19)
(includes o19 p19)(includes o19 p119)(includes o19 p143)

(waiting o20)
(includes o20 p11)(includes o20 p38)(includes o20 p41)(includes o20 p101)(includes o20 p134)

(waiting o21)
(includes o21 p16)(includes o21 p79)(includes o21 p87)(includes o21 p92)(includes o21 p167)

(waiting o22)
(includes o22 p3)(includes o22 p37)(includes o22 p58)(includes o22 p106)

(waiting o23)
(includes o23 p41)(includes o23 p58)(includes o23 p62)(includes o23 p78)(includes o23 p131)(includes o23 p164)

(waiting o24)
(includes o24 p10)(includes o24 p52)(includes o24 p76)(includes o24 p88)(includes o24 p99)(includes o24 p139)(includes o24 p162)(includes o24 p168)

(waiting o25)
(includes o25 p12)(includes o25 p102)

(waiting o26)
(includes o26 p2)(includes o26 p81)

(waiting o27)
(includes o27 p13)(includes o27 p55)(includes o27 p63)(includes o27 p87)

(waiting o28)
(includes o28 p73)(includes o28 p134)

(waiting o29)
(includes o29 p5)(includes o29 p7)(includes o29 p20)(includes o29 p48)(includes o29 p84)(includes o29 p111)(includes o29 p165)(includes o29 p169)

(waiting o30)
(includes o30 p34)(includes o30 p48)(includes o30 p65)(includes o30 p159)

(waiting o31)
(includes o31 p65)(includes o31 p124)(includes o31 p138)(includes o31 p140)(includes o31 p141)(includes o31 p152)(includes o31 p163)

(waiting o32)
(includes o32 p11)(includes o32 p27)(includes o32 p29)(includes o32 p57)

(waiting o33)
(includes o33 p86)(includes o33 p94)(includes o33 p127)(includes o33 p163)

(waiting o34)
(includes o34 p92)(includes o34 p108)(includes o34 p134)(includes o34 p151)

(waiting o35)
(includes o35 p30)(includes o35 p42)(includes o35 p83)(includes o35 p133)(includes o35 p163)

(waiting o36)
(includes o36 p8)(includes o36 p16)(includes o36 p53)(includes o36 p66)(includes o36 p169)

(waiting o37)
(includes o37 p68)(includes o37 p105)(includes o37 p111)

(waiting o38)
(includes o38 p27)(includes o38 p34)(includes o38 p45)(includes o38 p111)

(waiting o39)
(includes o39 p117)(includes o39 p123)(includes o39 p151)

(waiting o40)
(includes o40 p8)(includes o40 p48)(includes o40 p94)(includes o40 p163)

(waiting o41)
(includes o41 p20)(includes o41 p55)(includes o41 p64)(includes o41 p66)(includes o41 p67)(includes o41 p107)(includes o41 p155)

(waiting o42)
(includes o42 p81)(includes o42 p83)(includes o42 p94)(includes o42 p159)

(waiting o43)
(includes o43 p46)(includes o43 p56)(includes o43 p117)(includes o43 p141)(includes o43 p152)(includes o43 p159)

(waiting o44)
(includes o44 p5)(includes o44 p6)(includes o44 p23)(includes o44 p49)(includes o44 p68)(includes o44 p110)(includes o44 p122)(includes o44 p133)

(waiting o45)
(includes o45 p42)(includes o45 p53)(includes o45 p83)(includes o45 p97)(includes o45 p118)

(waiting o46)
(includes o46 p8)(includes o46 p10)(includes o46 p27)(includes o46 p33)(includes o46 p58)(includes o46 p60)(includes o46 p63)(includes o46 p110)(includes o46 p144)(includes o46 p148)

(waiting o47)
(includes o47 p10)(includes o47 p24)(includes o47 p47)(includes o47 p52)(includes o47 p97)(includes o47 p170)

(waiting o48)
(includes o48 p29)(includes o48 p76)(includes o48 p138)

(waiting o49)
(includes o49 p99)(includes o49 p103)(includes o49 p142)(includes o49 p149)

(waiting o50)
(includes o50 p11)(includes o50 p18)(includes o50 p43)(includes o50 p64)(includes o50 p100)(includes o50 p120)(includes o50 p149)(includes o50 p166)

(waiting o51)
(includes o51 p42)(includes o51 p62)(includes o51 p85)(includes o51 p99)(includes o51 p142)(includes o51 p149)

(waiting o52)
(includes o52 p24)(includes o52 p32)(includes o52 p53)(includes o52 p89)

(waiting o53)
(includes o53 p32)(includes o53 p36)(includes o53 p68)(includes o53 p161)

(waiting o54)
(includes o54 p2)(includes o54 p14)(includes o54 p31)(includes o54 p69)(includes o54 p135)(includes o54 p144)

(waiting o55)
(includes o55 p29)(includes o55 p33)(includes o55 p63)(includes o55 p64)(includes o55 p134)(includes o55 p155)

(waiting o56)
(includes o56 p13)(includes o56 p70)(includes o56 p119)(includes o56 p124)(includes o56 p134)

(waiting o57)
(includes o57 p27)(includes o57 p33)(includes o57 p54)(includes o57 p63)(includes o57 p102)(includes o57 p118)

(waiting o58)
(includes o58 p48)(includes o58 p66)(includes o58 p147)(includes o58 p151)

(waiting o59)
(includes o59 p55)(includes o59 p153)

(waiting o60)
(includes o60 p37)(includes o60 p88)(includes o60 p110)(includes o60 p134)(includes o60 p156)

(waiting o61)
(includes o61 p1)(includes o61 p33)(includes o61 p100)(includes o61 p108)(includes o61 p132)(includes o61 p164)(includes o61 p166)

(waiting o62)
(includes o62 p13)(includes o62 p60)(includes o62 p83)(includes o62 p121)(includes o62 p161)

(waiting o63)
(includes o63 p3)(includes o63 p63)(includes o63 p70)(includes o63 p101)(includes o63 p116)(includes o63 p118)(includes o63 p134)(includes o63 p138)

(waiting o64)
(includes o64 p7)(includes o64 p13)(includes o64 p69)

(waiting o65)
(includes o65 p15)(includes o65 p43)(includes o65 p94)

(waiting o66)
(includes o66 p5)(includes o66 p18)(includes o66 p55)(includes o66 p56)(includes o66 p95)(includes o66 p100)(includes o66 p105)(includes o66 p122)

(waiting o67)
(includes o67 p10)(includes o67 p75)(includes o67 p96)(includes o67 p117)(includes o67 p151)(includes o67 p158)

(waiting o68)
(includes o68 p18)(includes o68 p50)(includes o68 p54)(includes o68 p109)(includes o68 p150)

(waiting o69)
(includes o69 p15)(includes o69 p65)(includes o69 p114)(includes o69 p150)

(waiting o70)
(includes o70 p66)(includes o70 p73)(includes o70 p81)(includes o70 p105)(includes o70 p133)(includes o70 p134)(includes o70 p161)

(waiting o71)
(includes o71 p6)(includes o71 p10)(includes o71 p15)(includes o71 p25)(includes o71 p33)(includes o71 p110)(includes o71 p140)(includes o71 p163)

(waiting o72)
(includes o72 p15)(includes o72 p113)(includes o72 p119)(includes o72 p124)

(waiting o73)
(includes o73 p10)(includes o73 p23)(includes o73 p27)(includes o73 p84)(includes o73 p164)

(waiting o74)
(includes o74 p14)(includes o74 p48)(includes o74 p69)(includes o74 p95)(includes o74 p97)(includes o74 p100)

(waiting o75)
(includes o75 p20)(includes o75 p55)

(waiting o76)
(includes o76 p1)(includes o76 p29)(includes o76 p36)(includes o76 p41)(includes o76 p99)(includes o76 p137)(includes o76 p144)

(waiting o77)
(includes o77 p13)(includes o77 p70)(includes o77 p104)

(waiting o78)
(includes o78 p1)(includes o78 p13)(includes o78 p24)(includes o78 p63)(includes o78 p138)

(waiting o79)
(includes o79 p85)(includes o79 p96)(includes o79 p124)(includes o79 p149)

(waiting o80)
(includes o80 p5)(includes o80 p8)

(waiting o81)
(includes o81 p160)(includes o81 p167)

(waiting o82)
(includes o82 p6)(includes o82 p9)(includes o82 p89)(includes o82 p109)(includes o82 p132)(includes o82 p144)

(waiting o83)
(includes o83 p60)(includes o83 p81)(includes o83 p85)(includes o83 p110)(includes o83 p148)(includes o83 p149)(includes o83 p160)(includes o83 p167)

(waiting o84)
(includes o84 p1)(includes o84 p9)(includes o84 p33)(includes o84 p68)(includes o84 p85)(includes o84 p89)(includes o84 p93)(includes o84 p99)(includes o84 p104)

(waiting o85)
(includes o85 p2)(includes o85 p5)(includes o85 p15)(includes o85 p157)

(waiting o86)
(includes o86 p15)(includes o86 p22)(includes o86 p145)(includes o86 p166)

(waiting o87)
(includes o87 p34)(includes o87 p118)(includes o87 p130)(includes o87 p150)

(waiting o88)
(includes o88 p39)(includes o88 p71)(includes o88 p74)(includes o88 p167)

(waiting o89)
(includes o89 p31)(includes o89 p75)(includes o89 p127)(includes o89 p141)

(waiting o90)
(includes o90 p55)(includes o90 p63)(includes o90 p87)(includes o90 p115)(includes o90 p128)(includes o90 p132)(includes o90 p148)(includes o90 p167)

(waiting o91)
(includes o91 p41)(includes o91 p63)(includes o91 p73)(includes o91 p88)(includes o91 p165)

(waiting o92)
(includes o92 p19)(includes o92 p55)(includes o92 p56)(includes o92 p93)(includes o92 p123)(includes o92 p124)(includes o92 p132)(includes o92 p149)

(waiting o93)
(includes o93 p1)(includes o93 p49)(includes o93 p58)(includes o93 p128)(includes o93 p139)(includes o93 p167)

(waiting o94)
(includes o94 p44)

(waiting o95)
(includes o95 p68)(includes o95 p75)(includes o95 p123)(includes o95 p161)

(waiting o96)
(includes o96 p4)(includes o96 p112)(includes o96 p140)

(waiting o97)
(includes o97 p53)(includes o97 p78)(includes o97 p103)(includes o97 p115)

(waiting o98)
(includes o98 p18)(includes o98 p31)(includes o98 p71)

(waiting o99)
(includes o99 p8)(includes o99 p22)(includes o99 p43)(includes o99 p50)(includes o99 p65)(includes o99 p72)(includes o99 p96)(includes o99 p117)(includes o99 p130)(includes o99 p148)(includes o99 p149)

(waiting o100)
(includes o100 p7)(includes o100 p52)(includes o100 p63)

(waiting o101)
(includes o101 p46)(includes o101 p67)(includes o101 p88)(includes o101 p146)(includes o101 p166)

(waiting o102)
(includes o102 p22)(includes o102 p74)(includes o102 p107)(includes o102 p112)(includes o102 p115)(includes o102 p121)(includes o102 p136)(includes o102 p154)

(waiting o103)
(includes o103 p15)(includes o103 p43)(includes o103 p61)(includes o103 p71)(includes o103 p104)(includes o103 p126)(includes o103 p167)

(waiting o104)
(includes o104 p18)(includes o104 p44)(includes o104 p100)(includes o104 p102)(includes o104 p111)(includes o104 p125)

(waiting o105)
(includes o105 p40)(includes o105 p45)(includes o105 p49)(includes o105 p64)(includes o105 p96)(includes o105 p142)

(waiting o106)
(includes o106 p23)(includes o106 p64)(includes o106 p70)(includes o106 p113)

(waiting o107)
(includes o107 p6)(includes o107 p27)(includes o107 p60)

(waiting o108)
(includes o108 p28)(includes o108 p34)(includes o108 p65)(includes o108 p83)(includes o108 p106)

(waiting o109)
(includes o109 p72)(includes o109 p116)

(waiting o110)
(includes o110 p8)(includes o110 p9)(includes o110 p17)(includes o110 p67)(includes o110 p134)

(waiting o111)
(includes o111 p8)(includes o111 p13)(includes o111 p34)(includes o111 p82)(includes o111 p110)

(waiting o112)
(includes o112 p13)(includes o112 p32)(includes o112 p91)(includes o112 p108)(includes o112 p118)(includes o112 p126)

(waiting o113)
(includes o113 p8)(includes o113 p11)(includes o113 p24)(includes o113 p85)(includes o113 p100)

(waiting o114)
(includes o114 p1)(includes o114 p27)(includes o114 p42)(includes o114 p87)(includes o114 p106)(includes o114 p127)(includes o114 p168)

(waiting o115)
(includes o115 p62)(includes o115 p68)(includes o115 p110)(includes o115 p113)(includes o115 p114)(includes o115 p148)(includes o115 p157)(includes o115 p163)

(waiting o116)
(includes o116 p166)

(waiting o117)
(includes o117 p65)(includes o117 p112)(includes o117 p125)

(waiting o118)
(includes o118 p2)(includes o118 p10)(includes o118 p14)(includes o118 p21)(includes o118 p94)(includes o118 p127)

(waiting o119)
(includes o119 p30)(includes o119 p35)(includes o119 p37)(includes o119 p67)(includes o119 p126)(includes o119 p140)

(waiting o120)
(includes o120 p65)(includes o120 p117)(includes o120 p130)(includes o120 p141)

(waiting o121)
(includes o121 p3)(includes o121 p8)(includes o121 p13)(includes o121 p17)(includes o121 p71)(includes o121 p75)

(waiting o122)
(includes o122 p1)(includes o122 p52)(includes o122 p107)(includes o122 p168)

(waiting o123)
(includes o123 p28)(includes o123 p111)

(waiting o124)
(includes o124 p19)(includes o124 p24)(includes o124 p31)(includes o124 p71)(includes o124 p148)(includes o124 p163)

(waiting o125)
(includes o125 p36)(includes o125 p76)(includes o125 p124)(includes o125 p141)(includes o125 p170)

(waiting o126)
(includes o126 p21)(includes o126 p42)(includes o126 p96)(includes o126 p167)

(waiting o127)
(includes o127 p107)

(waiting o128)
(includes o128 p1)(includes o128 p9)(includes o128 p33)(includes o128 p131)(includes o128 p154)

(waiting o129)
(includes o129 p24)(includes o129 p60)(includes o129 p90)(includes o129 p105)(includes o129 p124)(includes o129 p137)(includes o129 p161)

(waiting o130)
(includes o130 p24)(includes o130 p30)(includes o130 p60)(includes o130 p67)(includes o130 p72)(includes o130 p148)

(waiting o131)
(includes o131 p30)(includes o131 p78)(includes o131 p98)(includes o131 p116)(includes o131 p131)

(waiting o132)
(includes o132 p73)(includes o132 p93)(includes o132 p102)(includes o132 p130)

(waiting o133)
(includes o133 p11)(includes o133 p30)(includes o133 p76)(includes o133 p104)

(waiting o134)
(includes o134 p21)(includes o134 p23)(includes o134 p59)(includes o134 p74)(includes o134 p109)(includes o134 p124)

(waiting o135)
(includes o135 p56)(includes o135 p119)(includes o135 p121)(includes o135 p146)

(waiting o136)
(includes o136 p83)(includes o136 p114)(includes o136 p120)(includes o136 p125)(includes o136 p145)

(waiting o137)
(includes o137 p5)(includes o137 p14)(includes o137 p15)(includes o137 p34)(includes o137 p78)(includes o137 p94)(includes o137 p95)(includes o137 p156)(includes o137 p163)

(waiting o138)
(includes o138 p31)(includes o138 p76)(includes o138 p100)

(waiting o139)
(includes o139 p40)(includes o139 p53)(includes o139 p77)(includes o139 p107)(includes o139 p127)

(waiting o140)
(includes o140 p5)(includes o140 p56)(includes o140 p82)(includes o140 p95)(includes o140 p104)(includes o140 p105)

(waiting o141)
(includes o141 p58)(includes o141 p63)(includes o141 p128)

(waiting o142)
(includes o142 p11)(includes o142 p34)(includes o142 p78)(includes o142 p136)

(waiting o143)
(includes o143 p32)(includes o143 p57)(includes o143 p58)(includes o143 p65)(includes o143 p86)(includes o143 p87)(includes o143 p95)(includes o143 p150)

(waiting o144)
(includes o144 p20)(includes o144 p22)(includes o144 p48)(includes o144 p74)(includes o144 p100)(includes o144 p124)(includes o144 p158)(includes o144 p170)

(waiting o145)
(includes o145 p118)

(waiting o146)
(includes o146 p8)(includes o146 p73)(includes o146 p159)

(waiting o147)
(includes o147 p56)(includes o147 p65)(includes o147 p73)(includes o147 p105)(includes o147 p121)(includes o147 p147)

(waiting o148)
(includes o148 p10)(includes o148 p20)(includes o148 p71)(includes o148 p90)(includes o148 p140)(includes o148 p145)(includes o148 p161)

(waiting o149)
(includes o149 p46)

(waiting o150)
(includes o150 p2)(includes o150 p65)(includes o150 p77)(includes o150 p91)(includes o150 p97)(includes o150 p142)

(waiting o151)
(includes o151 p19)(includes o151 p25)(includes o151 p49)(includes o151 p76)(includes o151 p137)(includes o151 p167)

(waiting o152)
(includes o152 p48)(includes o152 p145)

(waiting o153)
(includes o153 p44)(includes o153 p65)(includes o153 p74)(includes o153 p80)(includes o153 p94)

(waiting o154)
(includes o154 p42)(includes o154 p50)(includes o154 p51)(includes o154 p132)(includes o154 p133)

(waiting o155)
(includes o155 p97)(includes o155 p109)(includes o155 p132)(includes o155 p134)

(waiting o156)
(includes o156 p4)(includes o156 p46)(includes o156 p58)(includes o156 p63)(includes o156 p77)(includes o156 p118)(includes o156 p149)

(waiting o157)
(includes o157 p40)(includes o157 p50)(includes o157 p91)(includes o157 p134)(includes o157 p142)

(waiting o158)
(includes o158 p33)(includes o158 p51)

(waiting o159)
(includes o159 p52)

(waiting o160)
(includes o160 p83)(includes o160 p84)(includes o160 p119)(includes o160 p123)

(waiting o161)
(includes o161 p41)(includes o161 p44)(includes o161 p55)(includes o161 p78)(includes o161 p130)

(waiting o162)
(includes o162 p43)(includes o162 p89)(includes o162 p91)(includes o162 p118)

(waiting o163)
(includes o163 p16)(includes o163 p18)(includes o163 p38)(includes o163 p88)(includes o163 p163)(includes o163 p166)

(waiting o164)
(includes o164 p3)(includes o164 p27)(includes o164 p112)(includes o164 p140)(includes o164 p145)(includes o164 p148)

(waiting o165)
(includes o165 p51)(includes o165 p76)(includes o165 p98)(includes o165 p136)

(waiting o166)
(includes o166 p41)(includes o166 p98)(includes o166 p140)

(waiting o167)
(includes o167 p92)(includes o167 p99)(includes o167 p142)(includes o167 p168)

(waiting o168)
(includes o168 p9)(includes o168 p29)(includes o168 p42)(includes o168 p51)(includes o168 p60)(includes o168 p73)(includes o168 p129)

(waiting o169)
(includes o169 p12)(includes o169 p15)(includes o169 p31)(includes o169 p124)

(waiting o170)
(includes o170 p47)(includes o170 p57)(includes o170 p98)(includes o170 p107)(includes o170 p110)(includes o170 p133)(includes o170 p167)

(not-made p1)
(not-made p2)
(not-made p3)
(not-made p4)
(not-made p5)
(not-made p6)
(not-made p7)
(not-made p8)
(not-made p9)
(not-made p10)
(not-made p11)
(not-made p12)
(not-made p13)
(not-made p14)
(not-made p15)
(not-made p16)
(not-made p17)
(not-made p18)
(not-made p19)
(not-made p20)
(not-made p21)
(not-made p22)
(not-made p23)
(not-made p24)
(not-made p25)
(not-made p26)
(not-made p27)
(not-made p28)
(not-made p29)
(not-made p30)
(not-made p31)
(not-made p32)
(not-made p33)
(not-made p34)
(not-made p35)
(not-made p36)
(not-made p37)
(not-made p38)
(not-made p39)
(not-made p40)
(not-made p41)
(not-made p42)
(not-made p43)
(not-made p44)
(not-made p45)
(not-made p46)
(not-made p47)
(not-made p48)
(not-made p49)
(not-made p50)
(not-made p51)
(not-made p52)
(not-made p53)
(not-made p54)
(not-made p55)
(not-made p56)
(not-made p57)
(not-made p58)
(not-made p59)
(not-made p60)
(not-made p61)
(not-made p62)
(not-made p63)
(not-made p64)
(not-made p65)
(not-made p66)
(not-made p67)
(not-made p68)
(not-made p69)
(not-made p70)
(not-made p71)
(not-made p72)
(not-made p73)
(not-made p74)
(not-made p75)
(not-made p76)
(not-made p77)
(not-made p78)
(not-made p79)
(not-made p80)
(not-made p81)
(not-made p82)
(not-made p83)
(not-made p84)
(not-made p85)
(not-made p86)
(not-made p87)
(not-made p88)
(not-made p89)
(not-made p90)
(not-made p91)
(not-made p92)
(not-made p93)
(not-made p94)
(not-made p95)
(not-made p96)
(not-made p97)
(not-made p98)
(not-made p99)
(not-made p100)
(not-made p101)
(not-made p102)
(not-made p103)
(not-made p104)
(not-made p105)
(not-made p106)
(not-made p107)
(not-made p108)
(not-made p109)
(not-made p110)
(not-made p111)
(not-made p112)
(not-made p113)
(not-made p114)
(not-made p115)
(not-made p116)
(not-made p117)
(not-made p118)
(not-made p119)
(not-made p120)
(not-made p121)
(not-made p122)
(not-made p123)
(not-made p124)
(not-made p125)
(not-made p126)
(not-made p127)
(not-made p128)
(not-made p129)
(not-made p130)
(not-made p131)
(not-made p132)
(not-made p133)
(not-made p134)
(not-made p135)
(not-made p136)
(not-made p137)
(not-made p138)
(not-made p139)
(not-made p140)
(not-made p141)
(not-made p142)
(not-made p143)
(not-made p144)
(not-made p145)
(not-made p146)
(not-made p147)
(not-made p148)
(not-made p149)
(not-made p150)
(not-made p151)
(not-made p152)
(not-made p153)
(not-made p154)
(not-made p155)
(not-made p156)
(not-made p157)
(not-made p158)
(not-made p159)
(not-made p160)
(not-made p161)
(not-made p162)
(not-made p163)
(not-made p164)
(not-made p165)
(not-made p166)
(not-made p167)
(not-made p168)
(not-made p169)
(not-made p170)

(= (total-cost) 0)

)

(:goal
(and
(shipped o1)
(shipped o2)
(shipped o3)
(shipped o4)
(shipped o5)
(shipped o6)
(shipped o7)
(shipped o8)
(shipped o9)
(shipped o10)
(shipped o11)
(shipped o12)
(shipped o13)
(shipped o14)
(shipped o15)
(shipped o16)
(shipped o17)
(shipped o18)
(shipped o19)
(shipped o20)
(shipped o21)
(shipped o22)
(shipped o23)
(shipped o24)
(shipped o25)
(shipped o26)
(shipped o27)
(shipped o28)
(shipped o29)
(shipped o30)
(shipped o31)
(shipped o32)
(shipped o33)
(shipped o34)
(shipped o35)
(shipped o36)
(shipped o37)
(shipped o38)
(shipped o39)
(shipped o40)
(shipped o41)
(shipped o42)
(shipped o43)
(shipped o44)
(shipped o45)
(shipped o46)
(shipped o47)
(shipped o48)
(shipped o49)
(shipped o50)
(shipped o51)
(shipped o52)
(shipped o53)
(shipped o54)
(shipped o55)
(shipped o56)
(shipped o57)
(shipped o58)
(shipped o59)
(shipped o60)
(shipped o61)
(shipped o62)
(shipped o63)
(shipped o64)
(shipped o65)
(shipped o66)
(shipped o67)
(shipped o68)
(shipped o69)
(shipped o70)
(shipped o71)
(shipped o72)
(shipped o73)
(shipped o74)
(shipped o75)
(shipped o76)
(shipped o77)
(shipped o78)
(shipped o79)
(shipped o80)
(shipped o81)
(shipped o82)
(shipped o83)
(shipped o84)
(shipped o85)
(shipped o86)
(shipped o87)
(shipped o88)
(shipped o89)
(shipped o90)
(shipped o91)
(shipped o92)
(shipped o93)
(shipped o94)
(shipped o95)
(shipped o96)
(shipped o97)
(shipped o98)
(shipped o99)
(shipped o100)
(shipped o101)
(shipped o102)
(shipped o103)
(shipped o104)
(shipped o105)
(shipped o106)
(shipped o107)
(shipped o108)
(shipped o109)
(shipped o110)
(shipped o111)
(shipped o112)
(shipped o113)
(shipped o114)
(shipped o115)
(shipped o116)
(shipped o117)
(shipped o118)
(shipped o119)
(shipped o120)
(shipped o121)
(shipped o122)
(shipped o123)
(shipped o124)
(shipped o125)
(shipped o126)
(shipped o127)
(shipped o128)
(shipped o129)
(shipped o130)
(shipped o131)
(shipped o132)
(shipped o133)
(shipped o134)
(shipped o135)
(shipped o136)
(shipped o137)
(shipped o138)
(shipped o139)
(shipped o140)
(shipped o141)
(shipped o142)
(shipped o143)
(shipped o144)
(shipped o145)
(shipped o146)
(shipped o147)
(shipped o148)
(shipped o149)
(shipped o150)
(shipped o151)
(shipped o152)
(shipped o153)
(shipped o154)
(shipped o155)
(shipped o156)
(shipped o157)
(shipped o158)
(shipped o159)
(shipped o160)
(shipped o161)
(shipped o162)
(shipped o163)
(shipped o164)
(shipped o165)
(shipped o166)
(shipped o167)
(shipped o168)
(shipped o169)
(shipped o170)
))

(:metric minimize (total-cost))

)

