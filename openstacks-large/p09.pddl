(define (problem os-sequencedstrips-p210_3)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) 
(stacks-avail n0)

(waiting o1)
(includes o1 p10)(includes o1 p12)(includes o1 p36)(includes o1 p60)(includes o1 p138)

(waiting o2)
(includes o2 p15)(includes o2 p73)

(waiting o3)
(includes o3 p71)(includes o3 p132)(includes o3 p146)(includes o3 p150)(includes o3 p160)(includes o3 p187)(includes o3 p190)(includes o3 p197)

(waiting o4)
(includes o4 p78)(includes o4 p145)(includes o4 p154)(includes o4 p199)

(waiting o5)
(includes o5 p12)(includes o5 p63)(includes o5 p156)

(waiting o6)
(includes o6 p7)(includes o6 p105)(includes o6 p170)(includes o6 p187)(includes o6 p188)(includes o6 p196)(includes o6 p197)(includes o6 p202)

(waiting o7)
(includes o7 p71)(includes o7 p73)(includes o7 p106)(includes o7 p142)(includes o7 p166)

(waiting o8)
(includes o8 p19)(includes o8 p72)(includes o8 p105)(includes o8 p160)(includes o8 p200)(includes o8 p203)

(waiting o9)
(includes o9 p3)(includes o9 p12)(includes o9 p13)(includes o9 p56)(includes o9 p67)(includes o9 p72)(includes o9 p80)(includes o9 p103)(includes o9 p112)(includes o9 p160)(includes o9 p172)

(waiting o10)
(includes o10 p23)(includes o10 p42)(includes o10 p67)(includes o10 p69)(includes o10 p101)

(waiting o11)
(includes o11 p5)(includes o11 p35)(includes o11 p125)(includes o11 p131)(includes o11 p186)(includes o11 p203)

(waiting o12)
(includes o12 p75)(includes o12 p137)(includes o12 p152)(includes o12 p164)(includes o12 p186)(includes o12 p207)

(waiting o13)
(includes o13 p31)(includes o13 p80)(includes o13 p86)(includes o13 p121)(includes o13 p153)(includes o13 p154)(includes o13 p155)(includes o13 p165)(includes o13 p187)

(waiting o14)
(includes o14 p2)(includes o14 p15)(includes o14 p49)(includes o14 p72)(includes o14 p152)(includes o14 p210)

(waiting o15)
(includes o15 p8)(includes o15 p14)(includes o15 p34)(includes o15 p53)(includes o15 p114)(includes o15 p156)(includes o15 p177)

(waiting o16)
(includes o16 p5)(includes o16 p10)(includes o16 p23)(includes o16 p118)(includes o16 p185)

(waiting o17)
(includes o17 p4)(includes o17 p86)(includes o17 p97)(includes o17 p146)(includes o17 p186)(includes o17 p194)

(waiting o18)
(includes o18 p42)(includes o18 p86)(includes o18 p97)(includes o18 p105)(includes o18 p108)(includes o18 p156)(includes o18 p159)(includes o18 p199)

(waiting o19)
(includes o19 p14)(includes o19 p73)(includes o19 p77)(includes o19 p111)(includes o19 p137)(includes o19 p140)(includes o19 p162)(includes o19 p181)

(waiting o20)
(includes o20 p48)(includes o20 p62)(includes o20 p69)(includes o20 p88)(includes o20 p96)(includes o20 p173)(includes o20 p174)(includes o20 p185)

(waiting o21)
(includes o21 p24)(includes o21 p32)(includes o21 p36)(includes o21 p116)(includes o21 p118)(includes o21 p155)(includes o21 p172)(includes o21 p187)(includes o21 p202)(includes o21 p208)

(waiting o22)
(includes o22 p27)(includes o22 p46)(includes o22 p70)(includes o22 p141)

(waiting o23)
(includes o23 p20)(includes o23 p27)(includes o23 p36)(includes o23 p78)(includes o23 p96)(includes o23 p140)(includes o23 p177)(includes o23 p187)(includes o23 p196)

(waiting o24)
(includes o24 p5)(includes o24 p18)(includes o24 p23)(includes o24 p55)(includes o24 p90)(includes o24 p92)(includes o24 p136)

(waiting o25)
(includes o25 p2)(includes o25 p7)(includes o25 p12)(includes o25 p25)(includes o25 p51)(includes o25 p96)(includes o25 p160)(includes o25 p203)

(waiting o26)
(includes o26 p20)(includes o26 p43)(includes o26 p54)(includes o26 p131)(includes o26 p156)(includes o26 p157)(includes o26 p183)(includes o26 p208)

(waiting o27)
(includes o27 p11)(includes o27 p25)(includes o27 p73)(includes o27 p98)(includes o27 p160)(includes o27 p178)

(waiting o28)
(includes o28 p68)(includes o28 p84)(includes o28 p88)(includes o28 p111)(includes o28 p123)(includes o28 p154)(includes o28 p164)

(waiting o29)
(includes o29 p137)(includes o29 p149)(includes o29 p162)(includes o29 p196)(includes o29 p197)

(waiting o30)
(includes o30 p46)(includes o30 p91)(includes o30 p135)(includes o30 p137)(includes o30 p171)(includes o30 p188)

(waiting o31)
(includes o31 p4)(includes o31 p70)(includes o31 p147)

(waiting o32)
(includes o32 p14)(includes o32 p158)(includes o32 p196)(includes o32 p207)

(waiting o33)
(includes o33 p32)(includes o33 p125)(includes o33 p148)(includes o33 p149)

(waiting o34)
(includes o34 p46)(includes o34 p109)(includes o34 p117)(includes o34 p128)(includes o34 p137)(includes o34 p151)

(waiting o35)
(includes o35 p11)(includes o35 p154)(includes o35 p167)(includes o35 p176)

(waiting o36)
(includes o36 p39)(includes o36 p71)(includes o36 p88)(includes o36 p109)(includes o36 p113)(includes o36 p126)(includes o36 p132)(includes o36 p171)(includes o36 p179)(includes o36 p183)(includes o36 p200)

(waiting o37)
(includes o37 p3)(includes o37 p31)(includes o37 p52)(includes o37 p66)(includes o37 p97)(includes o37 p109)(includes o37 p121)

(waiting o38)
(includes o38 p3)(includes o38 p7)(includes o38 p16)(includes o38 p100)

(waiting o39)
(includes o39 p20)(includes o39 p73)(includes o39 p89)(includes o39 p197)(includes o39 p199)

(waiting o40)
(includes o40 p39)(includes o40 p71)(includes o40 p77)(includes o40 p91)

(waiting o41)
(includes o41 p13)(includes o41 p79)(includes o41 p87)

(waiting o42)
(includes o42 p26)(includes o42 p59)(includes o42 p71)(includes o42 p97)(includes o42 p141)(includes o42 p150)(includes o42 p157)

(waiting o43)
(includes o43 p70)(includes o43 p108)(includes o43 p119)(includes o43 p146)(includes o43 p166)(includes o43 p175)

(waiting o44)
(includes o44 p25)(includes o44 p28)(includes o44 p53)(includes o44 p72)(includes o44 p105)(includes o44 p135)(includes o44 p142)(includes o44 p184)(includes o44 p188)

(waiting o45)
(includes o45 p41)(includes o45 p115)(includes o45 p202)

(waiting o46)
(includes o46 p19)(includes o46 p70)(includes o46 p78)(includes o46 p157)(includes o46 p192)(includes o46 p198)

(waiting o47)
(includes o47 p35)(includes o47 p51)(includes o47 p63)(includes o47 p123)(includes o47 p171)

(waiting o48)
(includes o48 p96)(includes o48 p113)(includes o48 p120)(includes o48 p153)(includes o48 p167)

(waiting o49)
(includes o49 p5)(includes o49 p17)(includes o49 p74)(includes o49 p127)(includes o49 p174)(includes o49 p180)(includes o49 p196)

(waiting o50)
(includes o50 p25)(includes o50 p43)(includes o50 p50)(includes o50 p63)(includes o50 p144)(includes o50 p183)

(waiting o51)
(includes o51 p3)(includes o51 p32)(includes o51 p64)(includes o51 p96)

(waiting o52)
(includes o52 p90)(includes o52 p95)(includes o52 p129)(includes o52 p164)(includes o52 p167)(includes o52 p181)(includes o52 p205)

(waiting o53)
(includes o53 p25)(includes o53 p43)(includes o53 p51)(includes o53 p70)

(waiting o54)
(includes o54 p8)(includes o54 p24)(includes o54 p79)(includes o54 p105)(includes o54 p108)(includes o54 p154)(includes o54 p160)(includes o54 p205)(includes o54 p210)

(waiting o55)
(includes o55 p32)(includes o55 p38)(includes o55 p46)(includes o55 p116)(includes o55 p143)(includes o55 p205)

(waiting o56)
(includes o56 p51)(includes o56 p105)(includes o56 p181)(includes o56 p204)

(waiting o57)
(includes o57 p9)(includes o57 p26)(includes o57 p40)(includes o57 p48)(includes o57 p82)(includes o57 p135)(includes o57 p148)(includes o57 p164)(includes o57 p206)

(waiting o58)
(includes o58 p28)(includes o58 p43)(includes o58 p60)(includes o58 p63)(includes o58 p101)(includes o58 p118)(includes o58 p119)(includes o58 p126)(includes o58 p166)(includes o58 p186)(includes o58 p188)(includes o58 p191)

(waiting o59)
(includes o59 p14)(includes o59 p58)(includes o59 p62)(includes o59 p69)(includes o59 p74)(includes o59 p94)(includes o59 p202)

(waiting o60)
(includes o60 p5)(includes o60 p42)(includes o60 p59)(includes o60 p108)(includes o60 p120)(includes o60 p166)(includes o60 p202)

(waiting o61)
(includes o61 p46)(includes o61 p49)

(waiting o62)
(includes o62 p4)(includes o62 p20)(includes o62 p29)(includes o62 p32)(includes o62 p85)(includes o62 p142)(includes o62 p165)(includes o62 p199)

(waiting o63)
(includes o63 p21)(includes o63 p27)(includes o63 p42)(includes o63 p179)(includes o63 p181)(includes o63 p186)(includes o63 p205)(includes o63 p208)

(waiting o64)
(includes o64 p58)(includes o64 p196)(includes o64 p207)

(waiting o65)
(includes o65 p17)(includes o65 p102)(includes o65 p105)(includes o65 p146)(includes o65 p172)

(waiting o66)
(includes o66 p30)(includes o66 p32)(includes o66 p36)(includes o66 p52)(includes o66 p69)(includes o66 p99)(includes o66 p157)(includes o66 p185)(includes o66 p198)

(waiting o67)
(includes o67 p42)

(waiting o68)
(includes o68 p31)(includes o68 p82)(includes o68 p88)(includes o68 p175)(includes o68 p199)

(waiting o69)
(includes o69 p46)(includes o69 p50)(includes o69 p87)(includes o69 p153)(includes o69 p174)(includes o69 p199)

(waiting o70)
(includes o70 p4)(includes o70 p9)(includes o70 p41)(includes o70 p121)(includes o70 p154)(includes o70 p171)(includes o70 p187)

(waiting o71)
(includes o71 p39)(includes o71 p90)(includes o71 p95)(includes o71 p103)(includes o71 p109)(includes o71 p145)(includes o71 p146)

(waiting o72)
(includes o72 p17)(includes o72 p78)(includes o72 p101)(includes o72 p108)(includes o72 p122)(includes o72 p146)(includes o72 p180)(includes o72 p195)

(waiting o73)
(includes o73 p30)(includes o73 p135)(includes o73 p149)

(waiting o74)
(includes o74 p1)(includes o74 p3)(includes o74 p93)(includes o74 p111)(includes o74 p133)(includes o74 p158)(includes o74 p201)

(waiting o75)
(includes o75 p11)(includes o75 p35)(includes o75 p88)(includes o75 p101)(includes o75 p108)(includes o75 p128)

(waiting o76)
(includes o76 p24)(includes o76 p27)(includes o76 p44)(includes o76 p51)(includes o76 p55)(includes o76 p127)(includes o76 p145)

(waiting o77)
(includes o77 p53)(includes o77 p73)(includes o77 p118)(includes o77 p197)

(waiting o78)
(includes o78 p8)(includes o78 p11)(includes o78 p50)(includes o78 p54)(includes o78 p99)(includes o78 p104)(includes o78 p117)(includes o78 p165)(includes o78 p178)(includes o78 p195)

(waiting o79)
(includes o79 p14)(includes o79 p113)(includes o79 p140)(includes o79 p159)(includes o79 p175)(includes o79 p183)

(waiting o80)
(includes o80 p20)(includes o80 p34)(includes o80 p154)(includes o80 p180)

(waiting o81)
(includes o81 p13)(includes o81 p68)(includes o81 p77)(includes o81 p119)

(waiting o82)
(includes o82 p7)(includes o82 p40)(includes o82 p163)

(waiting o83)
(includes o83 p46)(includes o83 p149)(includes o83 p158)(includes o83 p193)

(waiting o84)
(includes o84 p4)(includes o84 p37)(includes o84 p152)(includes o84 p172)

(waiting o85)
(includes o85 p4)(includes o85 p36)

(waiting o86)
(includes o86 p63)(includes o86 p65)(includes o86 p72)(includes o86 p142)(includes o86 p199)(includes o86 p206)

(waiting o87)
(includes o87 p17)(includes o87 p175)

(waiting o88)
(includes o88 p13)(includes o88 p19)(includes o88 p126)(includes o88 p148)(includes o88 p182)

(waiting o89)
(includes o89 p1)(includes o89 p10)(includes o89 p34)(includes o89 p188)(includes o89 p206)(includes o89 p207)

(waiting o90)
(includes o90 p26)(includes o90 p35)(includes o90 p40)(includes o90 p41)(includes o90 p90)(includes o90 p98)(includes o90 p116)(includes o90 p122)

(waiting o91)
(includes o91 p10)(includes o91 p12)(includes o91 p39)(includes o91 p63)(includes o91 p84)(includes o91 p106)(includes o91 p156)

(waiting o92)
(includes o92 p3)(includes o92 p30)(includes o92 p133)(includes o92 p156)

(waiting o93)
(includes o93 p4)(includes o93 p52)(includes o93 p69)(includes o93 p109)(includes o93 p127)

(waiting o94)
(includes o94 p5)(includes o94 p13)(includes o94 p19)(includes o94 p20)(includes o94 p82)(includes o94 p184)(includes o94 p189)

(waiting o95)
(includes o95 p7)(includes o95 p10)(includes o95 p77)(includes o95 p115)

(waiting o96)
(includes o96 p3)(includes o96 p7)(includes o96 p74)(includes o96 p107)(includes o96 p196)

(waiting o97)
(includes o97 p10)(includes o97 p92)(includes o97 p136)(includes o97 p150)(includes o97 p185)

(waiting o98)
(includes o98 p28)(includes o98 p48)(includes o98 p137)(includes o98 p160)

(waiting o99)
(includes o99 p20)(includes o99 p87)(includes o99 p105)(includes o99 p202)

(waiting o100)
(includes o100 p45)(includes o100 p90)(includes o100 p132)(includes o100 p186)

(waiting o101)
(includes o101 p19)(includes o101 p38)(includes o101 p60)(includes o101 p115)(includes o101 p123)(includes o101 p130)

(waiting o102)
(includes o102 p16)(includes o102 p51)(includes o102 p66)(includes o102 p147)

(waiting o103)
(includes o103 p7)(includes o103 p15)(includes o103 p82)(includes o103 p128)(includes o103 p141)(includes o103 p152)(includes o103 p189)

(waiting o104)
(includes o104 p11)(includes o104 p38)(includes o104 p46)(includes o104 p93)(includes o104 p105)(includes o104 p163)

(waiting o105)
(includes o105 p26)(includes o105 p57)(includes o105 p86)

(waiting o106)
(includes o106 p100)(includes o106 p109)(includes o106 p201)

(waiting o107)
(includes o107 p40)(includes o107 p46)(includes o107 p81)(includes o107 p100)(includes o107 p205)

(waiting o108)
(includes o108 p9)(includes o108 p31)(includes o108 p32)(includes o108 p54)(includes o108 p119)(includes o108 p123)(includes o108 p157)(includes o108 p159)(includes o108 p189)

(waiting o109)
(includes o109 p19)(includes o109 p94)(includes o109 p107)(includes o109 p117)(includes o109 p133)(includes o109 p140)(includes o109 p154)(includes o109 p200)

(waiting o110)
(includes o110 p23)(includes o110 p49)(includes o110 p55)(includes o110 p91)

(waiting o111)
(includes o111 p17)(includes o111 p64)(includes o111 p67)(includes o111 p68)(includes o111 p70)(includes o111 p141)(includes o111 p187)

(waiting o112)
(includes o112 p43)(includes o112 p158)

(waiting o113)
(includes o113 p84)(includes o113 p98)(includes o113 p130)(includes o113 p165)

(waiting o114)
(includes o114 p10)(includes o114 p52)(includes o114 p61)(includes o114 p119)(includes o114 p127)(includes o114 p170)(includes o114 p205)

(waiting o115)
(includes o115 p32)(includes o115 p78)(includes o115 p86)(includes o115 p88)(includes o115 p108)(includes o115 p115)(includes o115 p158)(includes o115 p194)

(waiting o116)
(includes o116 p27)(includes o116 p79)(includes o116 p87)(includes o116 p137)(includes o116 p181)

(waiting o117)
(includes o117 p51)(includes o117 p125)

(waiting o118)
(includes o118 p4)(includes o118 p42)(includes o118 p53)(includes o118 p117)(includes o118 p135)(includes o118 p144)(includes o118 p146)(includes o118 p199)

(waiting o119)
(includes o119 p39)(includes o119 p56)(includes o119 p79)(includes o119 p108)(includes o119 p110)(includes o119 p123)(includes o119 p144)(includes o119 p178)

(waiting o120)
(includes o120 p38)(includes o120 p96)(includes o120 p149)(includes o120 p199)

(waiting o121)
(includes o121 p51)(includes o121 p52)(includes o121 p61)(includes o121 p91)(includes o121 p135)(includes o121 p188)(includes o121 p203)

(waiting o122)
(includes o122 p5)(includes o122 p48)(includes o122 p100)(includes o122 p129)(includes o122 p161)(includes o122 p164)(includes o122 p199)

(waiting o123)
(includes o123 p25)(includes o123 p29)(includes o123 p143)(includes o123 p148)(includes o123 p182)(includes o123 p193)

(waiting o124)
(includes o124 p11)(includes o124 p86)(includes o124 p89)(includes o124 p94)(includes o124 p105)(includes o124 p107)(includes o124 p111)(includes o124 p117)

(waiting o125)
(includes o125 p75)(includes o125 p148)(includes o125 p175)(includes o125 p200)

(waiting o126)
(includes o126 p29)(includes o126 p43)(includes o126 p50)(includes o126 p52)(includes o126 p70)(includes o126 p101)(includes o126 p120)(includes o126 p127)(includes o126 p142)(includes o126 p148)

(waiting o127)
(includes o127 p123)(includes o127 p158)(includes o127 p162)(includes o127 p203)

(waiting o128)
(includes o128 p5)(includes o128 p49)(includes o128 p106)(includes o128 p128)(includes o128 p141)(includes o128 p168)

(waiting o129)
(includes o129 p7)(includes o129 p50)(includes o129 p115)(includes o129 p118)(includes o129 p162)

(waiting o130)
(includes o130 p26)(includes o130 p52)(includes o130 p55)(includes o130 p84)(includes o130 p142)(includes o130 p191)(includes o130 p207)

(waiting o131)
(includes o131 p25)(includes o131 p43)(includes o131 p66)(includes o131 p102)(includes o131 p185)

(waiting o132)
(includes o132 p3)(includes o132 p27)(includes o132 p52)(includes o132 p146)(includes o132 p178)(includes o132 p200)

(waiting o133)
(includes o133 p18)(includes o133 p87)(includes o133 p123)(includes o133 p175)

(waiting o134)
(includes o134 p51)(includes o134 p54)(includes o134 p103)(includes o134 p175)

(waiting o135)
(includes o135 p52)(includes o135 p68)(includes o135 p88)

(waiting o136)
(includes o136 p4)(includes o136 p45)(includes o136 p51)(includes o136 p56)

(waiting o137)
(includes o137 p3)(includes o137 p105)

(waiting o138)
(includes o138 p20)(includes o138 p22)(includes o138 p62)(includes o138 p64)(includes o138 p95)(includes o138 p103)(includes o138 p122)(includes o138 p172)(includes o138 p183)

(waiting o139)
(includes o139 p27)(includes o139 p79)(includes o139 p110)(includes o139 p205)

(waiting o140)
(includes o140 p123)(includes o140 p139)(includes o140 p141)(includes o140 p147)

(waiting o141)
(includes o141 p18)(includes o141 p53)(includes o141 p92)(includes o141 p121)(includes o141 p123)

(waiting o142)
(includes o142 p9)(includes o142 p39)(includes o142 p164)(includes o142 p193)(includes o142 p199)

(waiting o143)
(includes o143 p5)(includes o143 p33)(includes o143 p38)(includes o143 p46)(includes o143 p51)(includes o143 p60)(includes o143 p79)(includes o143 p137)(includes o143 p146)(includes o143 p196)

(waiting o144)
(includes o144 p4)(includes o144 p22)(includes o144 p45)(includes o144 p68)(includes o144 p112)(includes o144 p134)(includes o144 p155)

(waiting o145)
(includes o145 p114)(includes o145 p194)

(waiting o146)
(includes o146 p22)(includes o146 p29)(includes o146 p43)(includes o146 p56)(includes o146 p62)(includes o146 p82)(includes o146 p100)(includes o146 p173)

(waiting o147)
(includes o147 p33)(includes o147 p84)(includes o147 p125)(includes o147 p136)(includes o147 p191)

(waiting o148)
(includes o148 p1)(includes o148 p5)(includes o148 p90)(includes o148 p111)(includes o148 p120)(includes o148 p124)(includes o148 p132)(includes o148 p164)(includes o148 p169)(includes o148 p197)

(waiting o149)
(includes o149 p9)(includes o149 p24)(includes o149 p80)(includes o149 p82)(includes o149 p99)(includes o149 p108)(includes o149 p148)(includes o149 p149)(includes o149 p167)(includes o149 p187)

(waiting o150)
(includes o150 p17)(includes o150 p26)(includes o150 p34)(includes o150 p54)(includes o150 p55)(includes o150 p74)(includes o150 p88)(includes o150 p159)

(waiting o151)
(includes o151 p45)(includes o151 p95)(includes o151 p130)

(waiting o152)
(includes o152 p40)(includes o152 p139)(includes o152 p203)

(waiting o153)
(includes o153 p21)(includes o153 p32)(includes o153 p131)(includes o153 p149)(includes o153 p178)

(waiting o154)
(includes o154 p16)(includes o154 p147)(includes o154 p196)

(waiting o155)
(includes o155 p23)(includes o155 p36)(includes o155 p40)(includes o155 p46)(includes o155 p48)(includes o155 p97)(includes o155 p104)(includes o155 p109)(includes o155 p117)

(waiting o156)
(includes o156 p24)(includes o156 p52)(includes o156 p69)(includes o156 p92)(includes o156 p123)(includes o156 p175)

(waiting o157)
(includes o157 p22)(includes o157 p26)(includes o157 p55)(includes o157 p60)(includes o157 p67)

(waiting o158)
(includes o158 p118)(includes o158 p121)(includes o158 p132)(includes o158 p191)(includes o158 p197)(includes o158 p209)

(waiting o159)
(includes o159 p11)(includes o159 p47)(includes o159 p50)(includes o159 p59)(includes o159 p68)(includes o159 p99)(includes o159 p128)(includes o159 p165)(includes o159 p173)

(waiting o160)
(includes o160 p11)(includes o160 p110)(includes o160 p135)(includes o160 p152)(includes o160 p178)

(waiting o161)
(includes o161 p2)(includes o161 p38)(includes o161 p40)(includes o161 p155)(includes o161 p196)

(waiting o162)
(includes o162 p81)(includes o162 p113)(includes o162 p120)(includes o162 p176)(includes o162 p194)

(waiting o163)
(includes o163 p14)(includes o163 p16)(includes o163 p49)(includes o163 p61)(includes o163 p83)(includes o163 p158)

(waiting o164)
(includes o164 p37)(includes o164 p48)(includes o164 p126)(includes o164 p150)(includes o164 p198)

(waiting o165)
(includes o165 p5)(includes o165 p6)(includes o165 p32)(includes o165 p58)(includes o165 p76)(includes o165 p78)(includes o165 p101)(includes o165 p106)(includes o165 p133)

(waiting o166)
(includes o166 p12)(includes o166 p17)(includes o166 p71)(includes o166 p126)(includes o166 p155)(includes o166 p207)

(waiting o167)
(includes o167 p9)(includes o167 p21)(includes o167 p110)(includes o167 p128)

(waiting o168)
(includes o168 p36)(includes o168 p37)(includes o168 p135)(includes o168 p150)(includes o168 p186)(includes o168 p187)

(waiting o169)
(includes o169 p9)(includes o169 p29)(includes o169 p99)(includes o169 p142)(includes o169 p180)(includes o169 p197)

(waiting o170)
(includes o170 p40)(includes o170 p49)

(waiting o171)
(includes o171 p4)(includes o171 p36)(includes o171 p37)(includes o171 p41)(includes o171 p67)(includes o171 p131)(includes o171 p145)(includes o171 p156)(includes o171 p170)

(waiting o172)
(includes o172 p17)(includes o172 p22)(includes o172 p99)(includes o172 p120)(includes o172 p156)(includes o172 p180)(includes o172 p206)

(waiting o173)
(includes o173 p35)(includes o173 p54)(includes o173 p72)(includes o173 p98)(includes o173 p100)(includes o173 p147)(includes o173 p184)(includes o173 p189)

(waiting o174)
(includes o174 p16)(includes o174 p110)(includes o174 p184)

(waiting o175)
(includes o175 p15)(includes o175 p21)(includes o175 p45)(includes o175 p94)(includes o175 p123)(includes o175 p149)(includes o175 p201)(includes o175 p208)

(waiting o176)
(includes o176 p12)(includes o176 p26)(includes o176 p48)(includes o176 p81)(includes o176 p96)(includes o176 p159)

(waiting o177)
(includes o177 p69)(includes o177 p108)(includes o177 p146)(includes o177 p201)

(waiting o178)
(includes o178 p15)(includes o178 p16)(includes o178 p33)(includes o178 p73)(includes o178 p105)(includes o178 p171)(includes o178 p174)

(waiting o179)
(includes o179 p21)(includes o179 p46)(includes o179 p65)(includes o179 p66)(includes o179 p91)(includes o179 p144)(includes o179 p187)(includes o179 p192)

(waiting o180)
(includes o180 p12)(includes o180 p88)(includes o180 p195)

(waiting o181)
(includes o181 p108)(includes o181 p167)(includes o181 p186)

(waiting o182)
(includes o182 p10)(includes o182 p45)(includes o182 p53)(includes o182 p69)(includes o182 p103)(includes o182 p114)(includes o182 p176)

(waiting o183)
(includes o183 p21)(includes o183 p42)(includes o183 p72)(includes o183 p124)(includes o183 p177)(includes o183 p208)

(waiting o184)
(includes o184 p14)(includes o184 p44)(includes o184 p151)(includes o184 p153)

(waiting o185)
(includes o185 p12)(includes o185 p17)(includes o185 p23)(includes o185 p48)(includes o185 p75)(includes o185 p112)(includes o185 p164)(includes o185 p174)(includes o185 p182)(includes o185 p209)

(waiting o186)
(includes o186 p22)(includes o186 p102)(includes o186 p112)(includes o186 p139)(includes o186 p140)(includes o186 p171)

(waiting o187)
(includes o187 p40)(includes o187 p70)(includes o187 p96)(includes o187 p140)(includes o187 p200)

(waiting o188)
(includes o188 p12)(includes o188 p19)(includes o188 p32)(includes o188 p60)(includes o188 p118)(includes o188 p161)(includes o188 p202)(includes o188 p210)

(waiting o189)
(includes o189 p38)(includes o189 p39)(includes o189 p102)(includes o189 p114)(includes o189 p181)(includes o189 p199)

(waiting o190)
(includes o190 p105)(includes o190 p174)(includes o190 p196)

(waiting o191)
(includes o191 p18)(includes o191 p149)(includes o191 p192)(includes o191 p200)

(waiting o192)
(includes o192 p102)(includes o192 p128)(includes o192 p163)(includes o192 p171)(includes o192 p192)

(waiting o193)
(includes o193 p43)(includes o193 p90)(includes o193 p102)(includes o193 p135)(includes o193 p165)(includes o193 p172)(includes o193 p185)(includes o193 p191)

(waiting o194)
(includes o194 p35)(includes o194 p201)

(waiting o195)
(includes o195 p11)(includes o195 p35)(includes o195 p43)(includes o195 p63)(includes o195 p70)(includes o195 p81)(includes o195 p82)(includes o195 p85)(includes o195 p140)(includes o195 p159)(includes o195 p199)

(waiting o196)
(includes o196 p14)(includes o196 p88)

(waiting o197)
(includes o197 p131)(includes o197 p140)(includes o197 p183)

(waiting o198)
(includes o198 p26)(includes o198 p37)(includes o198 p39)(includes o198 p134)(includes o198 p187)

(waiting o199)
(includes o199 p24)(includes o199 p37)(includes o199 p44)(includes o199 p67)(includes o199 p77)(includes o199 p84)(includes o199 p161)(includes o199 p167)(includes o199 p179)

(waiting o200)
(includes o200 p28)(includes o200 p100)(includes o200 p186)

(waiting o201)
(includes o201 p59)(includes o201 p67)

(waiting o202)
(includes o202 p10)(includes o202 p11)(includes o202 p99)(includes o202 p102)(includes o202 p147)(includes o202 p153)(includes o202 p154)(includes o202 p158)(includes o202 p171)

(waiting o203)
(includes o203 p36)(includes o203 p72)(includes o203 p98)(includes o203 p132)(includes o203 p146)

(waiting o204)
(includes o204 p126)(includes o204 p202)

(waiting o205)
(includes o205 p3)(includes o205 p10)(includes o205 p14)(includes o205 p27)(includes o205 p62)(includes o205 p128)(includes o205 p174)(includes o205 p199)

(waiting o206)
(includes o206 p56)(includes o206 p137)(includes o206 p196)

(waiting o207)
(includes o207 p38)(includes o207 p162)

(waiting o208)
(includes o208 p7)(includes o208 p82)(includes o208 p124)(includes o208 p158)(includes o208 p177)

(waiting o209)
(includes o209 p1)(includes o209 p41)(includes o209 p86)(includes o209 p101)(includes o209 p131)(includes o209 p170)(includes o209 p175)

(waiting o210)
(includes o210 p7)(includes o210 p66)(includes o210 p102)(includes o210 p117)(includes o210 p182)(includes o210 p202)

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
(not-made p171)
(not-made p172)
(not-made p173)
(not-made p174)
(not-made p175)
(not-made p176)
(not-made p177)
(not-made p178)
(not-made p179)
(not-made p180)
(not-made p181)
(not-made p182)
(not-made p183)
(not-made p184)
(not-made p185)
(not-made p186)
(not-made p187)
(not-made p188)
(not-made p189)
(not-made p190)
(not-made p191)
(not-made p192)
(not-made p193)
(not-made p194)
(not-made p195)
(not-made p196)
(not-made p197)
(not-made p198)
(not-made p199)
(not-made p200)
(not-made p201)
(not-made p202)
(not-made p203)
(not-made p204)
(not-made p205)
(not-made p206)
(not-made p207)
(not-made p208)
(not-made p209)
(not-made p210)

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
(shipped o171)
(shipped o172)
(shipped o173)
(shipped o174)
(shipped o175)
(shipped o176)
(shipped o177)
(shipped o178)
(shipped o179)
(shipped o180)
(shipped o181)
(shipped o182)
(shipped o183)
(shipped o184)
(shipped o185)
(shipped o186)
(shipped o187)
(shipped o188)
(shipped o189)
(shipped o190)
(shipped o191)
(shipped o192)
(shipped o193)
(shipped o194)
(shipped o195)
(shipped o196)
(shipped o197)
(shipped o198)
(shipped o199)
(shipped o200)
(shipped o201)
(shipped o202)
(shipped o203)
(shipped o204)
(shipped o205)
(shipped o206)
(shipped o207)
(shipped o208)
(shipped o209)
(shipped o210)
))

(:metric minimize (total-cost))

)

