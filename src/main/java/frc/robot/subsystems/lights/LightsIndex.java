package frc.robot.subsystems.lights;

public class LightsIndex {

    // A lookup table for 300 color names and their corresponding RGB values
    public static final String[][] COLORS = new String[][] {
        {"AliceBlue", "240, 248, 255"}, // #F0F8FF
        {"AntiqueWhite", "250, 235, 215"}, // #FAEBD7
        {"Aqua", "0, 255, 255"}, // #00FFFF
        {"Aquamarine", "127, 255, 212"}, // #7FFFD4
        {"Azure", "240, 255, 255"}, // #F0FFFF
        {"Beige", "245, 245, 220"}, // #F5F5DC
        {"Bisque", "255, 228, 196"}, // #FFE4C4
        {"Black", "0, 0, 0"}, // #000000
        {"BlanchedAlmond", "255, 235, 205"}, // #FFEBCD
        {"Blue", "0, 0, 255"}, // #0000FF
        {"BlueViolet", "138, 43, 226"}, // #8A2BE2
        {"Brown", "165, 42, 42"}, // #A52A2A
        {"BurlyWood", "222, 184, 135"}, // #DEB887
        {"CadetBlue", "95, 158, 160"}, // #5F9EA0
        {"Chartreuse", "127, 255, 0"}, // #7FFF00
        {"Chocolate", "210, 105, 30"}, // #D2691E
        {"Coral", "255, 127, 80"}, // #FF7F50
        {"CornflowerBlue", "100, 149, 237"}, // #6495ED
        {"Cornsilk", "255, 248, 220"}, // #FFF8DC
        {"Crimson", "220, 20, 60"}, // #DC143C
        {"Cyan", "0, 255, 255"}, // #00FFFF
        {"DarkBlue", "0, 0, 139"}, // #00008B
        {"DarkCyan", "0, 139, 139"}, // #008B8B
        {"DarkGoldenrod", "184, 134, 11"}, // #B8860B
        {"DarkGray", "169, 169, 169"}, // #A9A9A9
        {"DarkGreen", "0, 100, 0"}, // #006400
        {"DarkKhaki", "189, 183, 107"}, // #BDB76B
        {"DarkMagenta", "139, 0, 139"}, // #8B008B
        {"DarkOliveGreen", "85, 107, 47"}, // #556B2F
        {"DarkOrange", "255, 140, 0"}, // #FF8C00
        {"DarkOrchid", "153, 50, 204"}, // #9932CC
        {"DarkRed", "139, 0, 0"}, // #8B0000
        {"DarkSalmon", "233, 150, 122"}, // #E9967A
        {"DarkSeaGreen", "143, 188, 143"}, // #8FBC8F
        {"DarkSlateBlue", "72, 61, 139"}, // #483D8B
        {"DarkSlateGray", "47, 79, 79"}, // #2F4F4F
        {"DarkTurquoise", "0, 206, 209"}, // #00CED1
        {"DarkViolet", "148, 0, 211"}, // #9400D3
        {"DeepPink", "255, 20, 147"}, // #FF1493
        {"DeepSkyBlue", "0, 191, 255"}, // #00BFFF
        {"DimGray", "105, 105, 105"}, // #696969
        {"DodgerBlue", "30, 144, 255"}, // #1E90FF
        {"FireBrick", "178, 34, 34"}, // #B22222
        {"FloralWhite", "255, 250, 240"}, // #FFFAF0
        {"ForestGreen", "34, 139, 34"}, // #228B22
        {"Fuchsia", "255, 0, 255"}, // #FF00FF
        {"Gainsboro", "220, 220, 220"}, // #DCDCDC
        {"GhostWhite", "248, 248, 255"}, // #F8F8FF
        {"Gold", "255, 215, 0"}, // #FFD700
        {"Goldenrod", "218, 165, 32"}, // #DAA520
        {"Gray", "128, 128, 128"}, // #808080
        {"Green", "0, 128, 0"}, // #008000
        {"GreenYellow", "173, 255, 47"}, // #ADFF2F
        {"Honeydew", "240, 255, 240"}, // #F0FFF0
        {"HotPink", "255, 105, 180"}, // #FF69B4
        {"IndianRed", "205, 92, 92"}, // #CD5C5C
        {"Indigo", "75, 0, 130"}, // #4B0082
        {"Ivory", "255, 255, 240"}, // #FFFFF0
        {"Khaki", "240, 230, 140"}, // #F0E68C
        {"Lavender", "230, 230, 250"}, // #E6E6FA
        {"LavenderBlush", "255, 240, 245"}, // #FFF0F5
        {"LawnGreen", "124, 252, 0"}, // #7CFC00
        {"LemonChiffon", "255, 250, 205"}, // #FFFACD
        {"LightBlue", "173, 216, 230"}, // #ADD8E6
        {"LightCoral", "240, 128, 128"}, // #F08080
        {"LightCyan", "224, 255, 255"}, // #E0FFFF
        {"LightGoldenrodYellow", "250, 250, 210"}, // #FAFAD2
        {"LightGreen", "144, 238, 144"}, // #90EE90
        {"LightGray", "211, 211, 211"}, // #D3D3D3
        {"LightPink", "255, 182, 193"}, // #FFB6C1
        {"LightSalmon", "255, 160, 122"}, // #FFA07A
        {"LightSeaGreen", "32, 178, 170"}, // #20B2AA
        {"LightSkyBlue", "135, 206, 250"}, // #87CEFA
        {"LightSlateGray", "119, 136, 153"}, // #778899
        {"LightSteelBlue", "176, 224, 230"}, // #B0E0E6
        {"LightYellow", "255, 255, 224"}, // #FFFFE0
        {"Lime", "0, 255, 0"}, // #00FF00
        {"LimeGreen", "50, 205, 50"}, // #32CD32
        {"Linen", "250, 240, 230"}, // #FAF0E6
        {"Magenta", "255, 0, 255"}, // #FF00FF
        {"Maroon", "128, 0, 0"}, // #800000
        {"MediumAquamarine", "102, 205, 170"}, // #66CDAA
        {"MediumBlue", "0, 0, 205"}, // #0000CD
        {"MediumOrchid", "186, 85, 211"}, // #BA55D3
        {"MediumPurple", "147, 112, 219"}, // #9370DB
        {"MediumSeaGreen", "60, 179, 113"}, // #3CB371
        {"MediumSlateBlue", "123, 104, 238"}, // #7B68EE
        {"MediumSpringGreen", "0, 250, 154"}, // #00FA9A
        {"MediumTurquoise", "72, 209, 204"}, // #48D1CC
        {"MediumVioletRed", "199, 21, 133"}, // #C71585
        {"MidnightBlue", "25, 25, 112"}, // #191970
        {"MintCream", "245, 255, 250"}, // #F5FFFA
        {"MistyRose", "255, 228, 225"}, // #FFE4E1
        {"Moccasin", "255, 228, 181"}, // #FFE4B5
        {"NavajoWhite", "255, 222, 173"}, // #FFDEAD
        {"Navy", "0, 0, 128"}, // #000080
        {"OldLace", "253, 245, 230"}, // #FDF5E6
        {"Olive", "128, 128, 0"}, // #808000
        {"OliveDrab", "107, 142, 35"}, // #6B8E23
        {"Orange", "255, 165, 0"}, // #FFA500
        {"OrangeRed", "255, 69, 0"}, // #FF4500
        {"Orchid", "218, 112, 214"}, // #DA70D6
        {"PaleGoldenrod", "238, 232, 170"}, // #EEE8AA
        {"PaleGreen", "152, 251, 152"}, // #98FB98
        {"PaleTurquoise", "175, 238, 238"}, // #AFEEEE
        {"PaleVioletRed", "219, 112, 147"}, // #D87093
        {"PapayaWhip", "255, 239, 213"}, // #FFEFD5
        {"PeachPuff", "255, 218, 185"}, // #FFDAB9
        {"Peru", "205, 133, 63"}, // #CD853F
        {"Pink", "255, 192, 203"}, // #FFC0CB
        {"Plum", "221, 160, 221"}, // #DDA0DD
        {"PowderBlue", "176, 224, 230"}, // #B0E0E6
        {"Purple", "128, 0, 128"}, // #800080
        {"Red", "255, 0, 0"}, // #FF0000
        {"RosyBrown", "188, 143, 143"}, // #BC8F8F
        {"RoyalBlue", "65, 105, 225"}, // #4169E1
        {"SaddleBrown", "139, 69, 19"}, // #8B4513
        {"Salmon", "250, 128, 114"}, // #FA8072
        {"SandyBrown", "244, 164, 96"}, // #F4A460
        {"SeaGreen", "46, 139, 87"}, // #2E8B57
        {"Seashell", "255, 245, 238"}, // #FFF5EE
        {"Sienna", "160, 82, 45"}, // #A0522D
        {"Silver", "192, 192, 192"}, // #C0C0C0
        {"SkyBlue", "135, 206, 235"}, // #87CEEB
        {"SlateBlue", "106, 90, 205"}, // #6A5ACD
        {"SlateGray", "112, 128, 144"}, // #708090
        {"Snow", "255, 250, 250"}, // #FFFAFA
        {"SpringGreen", "0, 255, 127"}, // #00FF7F
        {"SteelBlue", "70, 130, 180"}, // #4682B4
        {"Tan", "210, 180, 140"}, // #D2B48C
        {"Teal", "0, 128, 128"}, // #008080
        {"Thistle", "216, 191, 216"}, // #D8BFD8
        {"Tomato", "255, 99, 71"}, // #FF6347
        {"Turquoise", "64, 224, 208"}, // #40E0D0
        {"Violet", "238, 130, 238"}, // #EE82EE
        {"Wheat", "245, 222, 179"}, // #F5DEB3
        {"White", "255, 255, 255"}, // #FFFFFF
        {"WhiteSmoke", "245, 245, 245"}, // #F5F5F5
        {"Yellow", "255, 255, 0"}, // #FFFF00
        {"YellowGreen", "154, 205, 50"}, // #9ACD32
        {"Amethyst", "153, 102, 204"}, // #9966CC
        {"Apricot", "251, 206, 177"}, // #FBCDB1
        {"Aquamarine", "127, 255, 212"}, // #7FFFD4
        {"ArcticLime", "208, 255, 20"}, // #D0FF14
        {"Auburn", "165, 42, 42"}, // #A52A2A
        {"Aurora", "255, 130, 171"}, // #FF82AB
        {"BarbiePink", "218, 41, 123"}, // #DA297B
        {"Bistre", "61, 43, 31"}, // #3D2B1F
        {"BlueGray", "128, 136, 153"}, // #808899
        {"BrightGreen", "102, 255, 0"}, // #66FF00
        {"Burgundy", "128, 0, 32"}, // #800020
        {"CadmiumRed", "227, 0, 34"}, // #E30022
        {"Celeste", "178, 255, 255"}, // #B2FFFF
        {"Chartreuse", "223, 255, 0"}, // #DFFF00
        {"Claret", "127, 23, 52"}, // #7F1734
        {"CoralRed", "255, 40, 0"}, // #FF2800
        {"CyanBlue", "0, 255, 255"}, // #00FFFF
        {"Daffodil", "255, 255, 49"}, // #FFFF31
        {"Emerald", "80, 200, 120"}, // #50C878
        {"FirebrickRed", "176, 34, 34"}, // #B02222
        {"Flame", "226, 85, 34"}, // #E25522
        {"FrenchRose", "247, 85, 153"}, // #F75599
        {"Grape", "111, 45, 168"}, // #6F2DA8
        {"Greenery", "136, 176, 75"}, // #88B04B
        {"Harlequin", "63, 255, 0"}, // #3FFF00
        {"HotMagenta", "255, 29, 206"}, // #FF1DCE
        {"LimePunch", "151, 255, 51"}, // #97FF33
        {"Mauve", "224, 176, 255"}, // #E0B0FF
        {"OliveGreen", "128, 128, 0"}, // #808000
        {"OrangePeel", "255, 159, 0"}, // #FF9F00
        {"OrchidPink", "242, 171, 197"}, // #F2ABC5
        {"PansyPurple", "135, 0, 178"}, // #8700B2
        {"RedOrange", "255, 69, 0"}, // #FF4500
        {"Raspberry", "227, 11, 93"}, // #E30B5C
        {"RichBlack", "0, 64, 64"}, // #004040
        {"Saffron", "255, 204, 51"}, // #FFCC33
        {"SalmonPink", "255, 145, 164"}, // #FF91A4
        {"SeaBlue", "0, 35, 56"}, // #002338
        {"Shamrock", "0, 255, 80"}, // #00FF50
        {"Sapphire", "15, 82, 186"}, // #0F52BA
        {"Scarlet", "255, 36, 0"}, // #FF2400
        {"Seafoam", "0, 255, 150"}, // #00FF96
        {"SunsetOrange", "253, 94, 83"} // #FD5E53
    };

    // Method to lookup the RGB value for a given color name
    public static String getRGB(String colorName) {
        for (String[] color : COLORS) {
            if (color[0].equalsIgnoreCase(colorName)) {
                return color[1];
            }
        }
        return "Color not found";  // Return a message if the color name is not found
    }
}
