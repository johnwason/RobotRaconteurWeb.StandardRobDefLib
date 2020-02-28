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
using YamlDotNet.Core;
using YamlDotNet.Serialization;
using RobotRaconteurWeb;
using YamlDotNet.Core.Events;
using System.Runtime.Serialization;

namespace RobotRaconteurWeb.InfoParser
{
    public class YamlVarValue : IYamlConvertible
    {
        public TypeDefinition type;

        public object value;

        public void Read(IParser parser, Type expectedType, ObjectDeserializer nestedObjectDeserializer)
        {
            if (!parser.TryConsume<MappingStart>(out var mapping))
            {
                throw new SerializationException("Invalid varvalue");
            }

            if (parser.TryConsume<MappingEnd>(out var _))
            {
                throw new SerializationException("Invalid varvalue");
            }
            
            var propertyName1 = parser.Consume<Scalar>().Value;
            if (propertyName1 != "type")
            {
                throw new SerializationException("Invalid varvalue: expected type");
            }

            var propertyValue1 = (string)nestedObjectDeserializer(typeof(string));

            if (parser.TryConsume<MappingEnd>(out var _))
            {
                throw new SerializationException("Invalid varvalue");
            }

            var propertyName2 = parser.Consume<Scalar>().Value;
            if (propertyName2 != "value")
            {
                throw new SerializationException("Invalid varvalue: expected value");
            }

            // TODO: Add more type conversions!
            switch(propertyValue1)
            {
                case "string":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.string_t;
                        value = (string)nestedObjectDeserializer(typeof(string));
                        break;
                    }
                case "double":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.double_t;
                        value = (double)nestedObjectDeserializer(typeof(double));
                        break;
                    }
                case "int32":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.int32_t;
                        value = (int)nestedObjectDeserializer(typeof(int));
                        break;
                    }
                case "uint32":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.uint32_t;
                        value = (uint)nestedObjectDeserializer(typeof(uint));
                        break;
                    }
                case "double[]":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.double_t;
                        type.ArrayType = DataTypes_ArrayTypes.array;
                        value = (double[])nestedObjectDeserializer(typeof(double[]));
                        break;
                    }
                case "int32[]":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.int32_t;
                        type.ArrayType = DataTypes_ArrayTypes.array;
                        value = (int[])nestedObjectDeserializer(typeof(int[]));
                        break;
                    }
                case "uint32[]":
                    {
                        type = new TypeDefinition();
                        type.Name = "value";
                        type.Type = DataTypes.uint32_t;
                        type.ArrayType = DataTypes_ArrayTypes.array;
                        value = (uint[])nestedObjectDeserializer(typeof(uint[]));
                        break;
                    }
                default:
                    throw new SerializationException($"Invalid varvalue: unknown type {propertyValue1}");
            }

            if (!parser.TryConsume<MappingEnd>(out var _))
            {
                throw new SerializationException("Invalid varvalue, extra fields found");
            }                       
        }

        public void Write(IEmitter emitter, ObjectSerializer nestedObjectSerializer)
        {
            throw new NotImplementedException();
        }
    }
}
