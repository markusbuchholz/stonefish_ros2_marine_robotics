/*   
    Copyright (c) 2024 Patryk Cieslak. All rights reserved.

    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#version 430

layout(location = 0) out vec4 fragColor;

uniform isampler2D texEventTimes;
// uniform sampler2D texCrossings;

void main() 
{
    int eventTime = texelFetch(texEventTimes, ivec2(gl_FragCoord.xy), 0).x;
    if(eventTime > 0)
        fragColor = vec4(0.0, 0.0, 1.0, 1.0);
    else if(eventTime < 0) 
        fragColor = vec4(1.0, 0.0, 0.0, 1.0);
    else
        fragColor = vec4(0.0);
    //fragColor = vec4((texelFetch(texCrossings, ivec2(gl_FragCoord.xy), 0).xxx + 50.0)/100.0, 1.0); // DEBUG
}