class OBJ:
    def __init__(self, filename):
        self.vertices = []
        self.edges = []
        self.faces = []

        with open(filename, 'r') as file:
            for line in file:
                if line.startswith('v '):
                    parts = line.strip().split()
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    self.vertices.append((x, y, z))
                elif line.startswith('l '):
                    parts = line.strip().split()
                    idx1 = int(parts[1]) - 1
                    idx2 = int(parts[2]) - 1
                    self.edges.append((idx1, idx2))
                elif line.startswith('f '):
                    parts = line.strip().split()
                    face = [int(idx) - 1 for idx in parts[1:]]
                    if len(face) == 3:
                        self.faces.append(tuple(face))
