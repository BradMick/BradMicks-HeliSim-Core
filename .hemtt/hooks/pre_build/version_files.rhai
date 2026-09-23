// Generates version.hpp for each addon from the project version plus the git hash.

// No .git directory to read - a source zip, or a submodule checkout where .git is a file
// pointing elsewhere. That build is versioned without a hash rather than failing.
let head_file = HEMTT_RFS.join(".git").join("HEAD");
let head = "";
if head_file.is_file() {
    head = head_file.open_file().read();
    head.trim();
}

let full_hash = "";
if head == "" {
    // No hash available - falls through to the zero hash below.
} else if head.contains("ref: ") {
    let ref_path = head;
    ref_path.replace("ref: ", "");
    ref_path.trim();
    let parts = ref_path.split("/");
    let ref_file = HEMTT_RFS.join(".git");
    for part in parts {
        ref_file = ref_file.join(part);
    }
    if ref_file.is_file() {
        full_hash = ref_file.open_file().read();
        full_hash.trim();
    } else {
        let packed_path = HEMTT_RFS.join(".git").join("packed-refs");
        if packed_path.is_file() {
            let packed = packed_path.open_file().read();
            for line in packed.split("\n") {
                let l = line;
                l.trim();
                if l.len() > 0 && !l.contains("#") && l.contains(ref_path) {
                    full_hash = l.split(" ")[0];
                    break;
                }
            }
        }
    }
} else {
    full_hash = head;
}

// Not a git checkout yet - version without a hash rather than failing the build.
let hash = "00000000";
if full_hash.len() >= 8 {
    hash = full_hash.sub_string(0, 8);
}

let ver = HEMTT.project().version();
let ver_str = ver.major().to_string() + "." + ver.minor().to_string() + "." + ver.patch().to_string() + "." + ver.build().to_string();
let ver_ar = ver.major().to_string() + ", " + ver.minor().to_string() + ", " + ver.patch().to_string() + ", " + ver.build().to_string();

let lines = "version = \"" + ver_str + "\";\n";
lines += "versionStr = \"" + ver_str + "\";\n";
lines += "versionAr[] = {" + ver_ar + "};\n";
lines += "hash = \"" + hash + "\";\n";

let addons_path = HEMTT_VFS.join("addons");
let rfs_addons_path = HEMTT_RFS.join("addons");
let count = 0;

for entry in addons_path.list() {
    if entry.is_dir() {
        let name = entry.file_name();
        entry.join("version.hpp").create_file().write(lines);
        rfs_addons_path.join(name).join("version.hpp").create_file().write(lines);
        count += 1;
    }
}

print("Generated version.hpp for " + count.to_string() + " addon(s) (hash: " + hash + ")");
