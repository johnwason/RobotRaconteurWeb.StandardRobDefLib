// Copyright 2020 Rensselaer Polytechnic Institute
//                Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace RobotRaconteurWeb.InfoParser
{
    public class YamlDataType
    {
        public RobotRaconteurWeb.TypeDefinition data_type { get; set; }
        public static explicit operator YamlDataType(string s)
        {
            var d = new RobotRaconteurWeb.TypeDefinition();
            d.FromString(s);
            return new YamlDataType() { data_type = d };
        }

        public void CopyTo(com.robotraconteur.datatype.DataType info)
        {
            info.name = data_type.Name ?? "";
            info.type_code = (com.robotraconteur.datatype.DataTypeCode)data_type.Type;
            info.type_string = data_type.TypeString ?? "";
            info.array_type_code = (com.robotraconteur.datatype.ArrayTypeCode)data_type.ArrayType;
            info.array_var_len = data_type.ArrayVarLength;
            info.array_len = data_type.ArrayLength.Select(x=>(uint)x).ToArray();
            info.container_type_code = (com.robotraconteur.datatype.ContainerTypeCode)data_type.ContainerType;            
        }

        public com.robotraconteur.datatype.DataType ToRRInfo()
        {
            var info = new com.robotraconteur.datatype.DataType();
            CopyTo(info);
            return info;
        }

    }
}
