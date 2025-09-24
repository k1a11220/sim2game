#!/usr/bin/env python3
"""
Merge multiple OBJ files into a single file with separate object groups
"""

def merge_obj_files(drone_file, tung_file, output_file):
    with open(output_file, 'w') as out:
        # Write material library reference
        out.write("mtllib phong3SG.mtl\n\n")

        # Process drone OBJ
        out.write("# Drone object\n")
        out.write("o drone\n")
        out.write("usemtl phong3SG\n")

        v_offset = 0
        vt_offset = 0
        vn_offset = 0

        with open(drone_file, 'r') as f:
            for line in f:
                if line.startswith('mtllib') or line.startswith('usemtl'):
                    continue  # Skip material lines
                elif line.startswith('o '):
                    continue  # Skip object lines
                else:
                    out.write(line)
                    if line.startswith('v '):
                        v_offset += 1
                    elif line.startswith('vt '):
                        vt_offset += 1
                    elif line.startswith('vn '):
                        vn_offset += 1

        # Process tung OBJ with vertex offset
        out.write("\n# Tung object\n")
        out.write("o tung\n")
        out.write("usemtl Sahur\n")

        with open(tung_file, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    out.write(line)
                elif line.startswith('vt '):
                    out.write(line)
                elif line.startswith('vn '):
                    out.write(line)
                elif line.startswith('f '):
                    # Adjust face indices
                    parts = line.strip().split()
                    new_parts = ['f']
                    for part in parts[1:]:
                        indices = part.split('/')
                        new_indices = []
                        if len(indices) > 0 and indices[0]:
                            new_indices.append(str(int(indices[0]) + v_offset))
                        else:
                            new_indices.append('')
                        if len(indices) > 1 and indices[1]:
                            new_indices.append(str(int(indices[1]) + vt_offset))
                        else:
                            new_indices.append('')
                        if len(indices) > 2 and indices[2]:
                            new_indices.append(str(int(indices[2]) + vn_offset))
                        new_parts.append('/'.join(new_indices))
                    out.write(' '.join(new_parts) + '\n')

if __name__ == "__main__":
    merge_obj_files(
        "X2_lowpoly.obj",
        "tung_nomtl.obj",
        "merged_model.obj"
    )
    print("Merged OBJ created: merged_model.obj")