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
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using com.robotraconteur.identifier;
using com.robotraconteur.uuid;
using RobotRaconteurWeb;

namespace RobotRaconteurWeb.InfoParser
{

    public class YamlUuid
    {
        public string source_string { get; set; }
        public byte[] uuid_bytes { get; set; } = new byte[16];

        public YamlUuid(string str_uuid)
        {
            source_string = str_uuid;
            if (!TryParse(str_uuid, out byte[] uuid_bytes))
            {
                throw new ArgumentException("Invalid UUID string in yaml config file");
            }
            this.uuid_bytes = uuid_bytes;
        }


        // Taken from RobotRaconteurWeb.NodeID
        protected static bool TryParse(string stringid, out byte[] bytes)
        {
            if (stringid == "{0}")
            {
                bytes = new byte[16];
                return true;
            }

            bytes = null;
            Regex r = new Regex(@"\{?([a-fA-F0-9]{8})-([a-fA-F0-9]{4})-([a-fA-F0-9]{4})-([a-fA-F0-9]{4})-([a-fA-F0-9]{12})\}?");
            var res = r.Match(stringid);
            if (!res.Success) return false;
            string res1 = "";
            for (int i = 1; i < 6; i++) res1 += res.Groups[i].Value;
            bytes = new byte[16];
            for (int i = 0; i < 16; i++)
            {
                bytes[i] = Convert.ToByte(res1.Substring(i * 2, 2), 16);
            }
            return true;
        }

        public static explicit operator YamlUuid(string s) => new YamlUuid(s);

        public void CopyTo(ref com.robotraconteur.uuid.UUID uuid)
        {
            uuid.uuid_bytes = (byte[])uuid_bytes.Clone() ?? new byte[16];
        }

        public com.robotraconteur.uuid.UUID ToRRInfo()
        {
            var uuid = new com.robotraconteur.uuid.UUID();
            CopyTo(ref uuid);
            return uuid;        
        }
    }

    public class YamlIdentifier
    {
        public string name { get; set; }
        public YamlUuid uuid { get; set; }

        public static explicit operator YamlIdentifier(string s) => new YamlIdentifier() { name = s };

        public void CopyTo(com.robotraconteur.identifier.Identifier id)
        {
            id.name = name;
            if (uuid == null)
            {
                id.uuid = new com.robotraconteur.uuid.UUID();
                id.uuid.uuid_bytes = new byte[16];
            }
            else
            {
                id.uuid = uuid.ToRRInfo();
            }
        }

        public com.robotraconteur.identifier.Identifier ToRRInfo()
        {
            var id = new com.robotraconteur.identifier.Identifier();
            CopyTo(id);
            return id;
        }
    }

    public class YamlResourceIdentifier
    {
        public YamlIdentifier bucket { get; set; }
        public string key { get; set; }

        public void CopyTo(com.robotraconteur.resource.ResourceIdentifier id)
        {
            id.bucket = bucket?.ToRRInfo();
            id.key = key ?? "";
        }

        public com.robotraconteur.resource.ResourceIdentifier ToRRInfo()
        {
            var id = new com.robotraconteur.resource.ResourceIdentifier();
            CopyTo(id);
            return id;
        }
    }

    // This file is largely based on RobotRaconteurWeb.LocalTransport NodeID generation and locking


    /// <summary>
    /// Utility class to generate a local UUID for an identifier name
    /// and persist the generated UUID
    /// </summary>
    public static class LocalIdentifiersManager
    {
        public static  LocalIdentifierLock GetIdentifierForNameAndLock(string category, string name)
        {
            NodeID nodeid = null;

            if (!Regex.IsMatch(name, "^[a-zA-Z][a-zA-Z0-9_\\.\\-]*$"))
            {
                throw new ArgumentException("\"" + name + "\" is an invalid identifier name");
            }

            category = category.ToLowerInvariant();

            var storage_path = GetUserIdentifierStoragePath();
            Directory.CreateDirectory(Path.Combine(storage_path, category));
            string p = Path.Combine(storage_path, category, name);

            bool is_windows = RuntimeInformation.IsOSPlatform(OSPlatform.Windows);


            LocalIdentifierFD fd = null;
            LocalIdentifierFD fd_run = null;
            try
            {
                if (is_windows)
                {
                    fd = new LocalIdentifierFD();

                    int error_code;
                    if (!fd.OpenLockWrite(p, false, out error_code))
                    {
                        if (error_code == 32)
                        {
                            throw new InvalidOperationException("Identifier already in use");
                        }
                        throw new SystemResourceException("Could not initialize local identifiers");
                    }
                }
                else
                {
                    string p_lock = Path.Combine(GetUserRunPath(), "identifiers");

                    Directory.CreateDirectory(p_lock);

                    p_lock = Path.Combine(p_lock, category);
                    Directory.CreateDirectory(p_lock);

                    p_lock = Path.Combine(p_lock, name + ".pid");

                    fd_run = new LocalIdentifierFD();

                    int open_run_err;
                    if (!fd_run.OpenLockWrite(p_lock, false, out open_run_err))
                    {
                        if (open_run_err == (int)Mono.Unix.Native.Errno.ENOLCK)
                        {
                            throw new InvalidOperationException("Identifier already in use");
                        }
                        throw new SystemResourceException("Could not initialize local identifiers");
                    }

                    string pid_str = Process.GetCurrentProcess().Id.ToString();
                    if (!fd_run.Write(pid_str))
                    {
                        throw new SystemResourceException("Could not initialize local identifiers");
                    }

                    fd = new LocalIdentifierFD();

                    int open_err;
                    if (!fd.OpenLockWrite(p, false, out open_err))
                    {
                        if (open_err == (int)Mono.Unix.Native.Errno.EROFS)
                        {
                            open_err = 0;
                            if (!fd.OpenRead(p, out open_err))
                            {
                                throw new InvalidOperationException("LocalTransport NodeID not set on read only filesystem");
                            }
                        }
                        else
                        {
                            throw new SystemResourceException("Could not initialize LocalTransport server");
                        }
                    }
                }
                int len = fd.FileLen;

                if (len == 0 || len == -1 || len > 16 * 1024)
                {
                    nodeid = NodeID.NewUniqueID();
                    string dat = nodeid.ToString();
                    fd.Write(dat);
                }
                else
                {
                    string nodeid_str;
                    fd.Read(out nodeid_str);
                    try
                    {
                        nodeid_str = nodeid_str.Trim();
                        nodeid = new NodeID(nodeid_str);
                    }
                    catch (Exception)
                    {
                        throw new IOException("Error in identifier settings file");
                    }
                }

                var ident_uuid = new UUID() { uuid_bytes = nodeid.ToByteArray() };
                var ident = new Identifier() { name = name, uuid = ident_uuid };


                if (is_windows)
                {
                    return new LocalIdentifierLock(ident, fd);
                }
                else
                {
                    fd?.Dispose();
                    return new LocalIdentifierLock(ident, fd_run);
                }
            }
            catch (Exception)
            {
                fd?.Dispose();
                fd_run?.Dispose();
                throw;
            }
        }

        private static string GetUserIdentifierStoragePath()
        {
            try
            {
                var p = Path.Combine(GetUserDataPath(), "identifiers");
                if (!Directory.Exists(p))
                {
                    Directory.CreateDirectory(p);
                }

                return p;
            }
            catch (Exception ee)
            {
                throw new SystemResourceException("Could not activate system for local identifiers: " + ee.Message);
            }
        }

        private static string GetUserDataPath()
        {
            try
            {
                if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                {

                    var p = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData, Environment.SpecialFolderOption.Create);

                    var p1 = Path.Combine(p, "RobotRaconteur");
                    if (!Directory.Exists(p1))
                    {
                        Directory.CreateDirectory(p1);
                    }
                    return p1;
                }
                else
                {
                    var p1 = Path.Combine(Environment.GetEnvironmentVariable("HOME"), ".config/RobotRaconteur");
                    if (!Directory.Exists(p1))
                    {
                        Directory.CreateDirectory(p1);
                    }
                    return p1;
                }
            }

            catch (Exception ee)
            {
                throw new SystemResourceException("Could not activate system for local identifiers: " + ee.Message);
            }
        }

        private static int check_mkdir_res(int res)
        {
            if (Mono.Unix.Native.Syscall.GetLastError() == Mono.Unix.Native.Errno.EEXIST)
            {
                return 0;
            }
            return res;
        }
        private static string GetUserRunPath()
        {
            try
            {
                if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                {
                    var p = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData, Environment.SpecialFolderOption.Create);

                    var p1 = Path.Combine(p, "RobotRaconteur", "run");
                    if (!Directory.Exists(p1))
                    {
                        Directory.CreateDirectory(p1);
                    }
                    return p1;
                }
                else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                {
                    uint u = Mono.Unix.Native.Syscall.getuid();

                    string path;
                    if (u == 0)
                    {
                        path = "/var/run/robotraconteur/root/";
                        if (check_mkdir_res(Mono.Unix.Native.Syscall.mkdir(path, Mono.Unix.Native.FilePermissions.S_IRUSR
                            | Mono.Unix.Native.FilePermissions.S_IWUSR | Mono.Unix.Native.FilePermissions.S_IXUSR)) < 0)
                        {
                            throw new SystemResourceException("Could not create root run directory");
                        }
                    }
                    else
                    {
                        string path1 = Environment.GetEnvironmentVariable("TMPDIR");
                        if (path1 == null)
                        {
                            throw new SystemResourceException("Could not determine TMPDIR");
                        }

                        path = Path.GetDirectoryName(path1.TrimEnd(Path.DirectorySeparatorChar));
                        path = Path.Combine(path, "C");
                        if (!Directory.Exists(path))
                        {
                            throw new SystemResourceException("Could not determine user cache dir");
                        }

                        path = Path.Combine(path, "robotraconteur");
                        if (check_mkdir_res(Mono.Unix.Native.Syscall.mkdir(path, Mono.Unix.Native.FilePermissions.S_IRUSR
                            | Mono.Unix.Native.FilePermissions.S_IWUSR | Mono.Unix.Native.FilePermissions.S_IXUSR)) < 0)
                        {
                            throw new SystemResourceException("Could not create user run directory");
                        }
                    }
                    return path;
                }
                else
                {
                    uint u = Mono.Unix.Native.Syscall.getuid();

                    string path;
                    if (u == 0)
                    {
                        path = "/var/run/robotraconteur/root/";
                        if (check_mkdir_res(Mono.Unix.Native.Syscall.mkdir(path, Mono.Unix.Native.FilePermissions.S_IRUSR
                            | Mono.Unix.Native.FilePermissions.S_IWUSR | Mono.Unix.Native.FilePermissions.S_IXUSR)) < 0)
                        {
                            throw new SystemResourceException("Could not create root run directory");
                        }
                    }
                    else
                    {
                        path = Environment.GetEnvironmentVariable("XDG_RUNTIME_DIR");

                        if (path == null)
                        {
                            path = String.Format("/var/run/user/{0}/", u);
                        }

                        path = Path.Combine(path, "robotraconteur");
                        if (check_mkdir_res(Mono.Unix.Native.Syscall.mkdir(path, Mono.Unix.Native.FilePermissions.S_IRUSR
                            | Mono.Unix.Native.FilePermissions.S_IWUSR | Mono.Unix.Native.FilePermissions.S_IXUSR)) < 0)
                        {
                            throw new SystemResourceException("Could not create user run directory");
                        }
                    }
                    return path;
                }
            }
            catch (Exception ee)
            {
                throw new SystemResourceException("Could not activate system for local transport: " + ee.Message);
            }
        }
    }

    public class LocalIdentifierLock : IDisposable
    {
        internal LocalIdentifierLock(Identifier id, LocalIdentifierFD fd)
        {
            this.fd = fd;
            this.Identifier = id;
        }

        LocalIdentifierFD fd;
        public Identifier Identifier { get; }

        public void Dispose()
        {
            fd?.Dispose();
        }
    }

    public class LocalIdentifierLocks : IDisposable
    {
        public LocalIdentifierLock[] IdentifierLocks { get; private set; }

        public LocalIdentifierLocks(LocalIdentifierLock[] locks)
        {
            IdentifierLocks = locks;
        }

        public LocalIdentifierLocks(LocalIdentifierLock lock_)
        {
            IdentifierLocks = new LocalIdentifierLock[] { lock_ };
        }

        public void Dispose()
        {
            if (IdentifierLocks != null)
            {
                foreach (var d in IdentifierLocks)
                {
                    d?.Dispose();
                }
            }
        }

        public void Release()
        {
            IdentifierLocks = null;
        }
    }


    // Based on RobotRaconteurWeb.LocalTransportFD
    class LocalIdentifierFD : IDisposable
    {
        FileStream f;

        public Dictionary<string, string> Info { get; set; }

        public LocalIdentifierFD()
        {

        }

        public bool OpenRead(string path, out int error_code)
        {
            try
            {
                var h = File.Open(path, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
                f = h;
                error_code = 0;
                return true;
            }
            catch (Exception ee)
            {
                error_code = 0xFFFF & ee.HResult;
                return false;
            }

        }

        public bool OpenLockWrite(string path, bool delete_on_close, out int error_code)
        {
            FileOptions file_options = default(FileOptions);
            if (delete_on_close)
            {
                file_options |= FileOptions.DeleteOnClose;
            }

            try
            {
                if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                {
                    var h = new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.Read, 4096, file_options);
                    f = h;
                }
                else
                {
                    var h = new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.Read, 1024, file_options);
                    h.Seek(0, SeekOrigin.Begin);
                    try
                    {
                        h.Lock(0, 0);
                    }
                    catch (Exception)
                    {
                        h.Dispose();
                        throw;
                    }

                    f = h;
                }

                error_code = 0;
                return true;
            }
            catch (Exception ee)
            {
                error_code = 0xFFFF & ee.HResult;
                return false;
            }
        }

        public bool Read(out string data)
        {
            try
            {
                f.Seek(0, SeekOrigin.Begin);
                long len = f.Length;
                var reader = new StreamReader(f);
                data = reader.ReadToEnd();
                return true;
            }
            catch (Exception)
            {
                data = null;
                return false;
            }
        }
                
        public bool Write(string data)
        {
            try
            {
                var w = new StreamWriter(f);
                w.Write(data);
                w.Flush();
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }
        
        public int FileLen
        {
            get
            {
                return (int)f.Length;
            }
        }

        public void Dispose()
        {
            f?.Dispose();
        }
    }




}
