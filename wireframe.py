import math

#Clase básica que representa un punto (nodo) en el espacio 3D
class Node:
    def __init__(self, coordinates):
        coordinates = list(coordinates)
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]


class Wireframe:
    def __init__(self):
        self.nodes = []                        #Lista de nodos (coordenadas en el espacio)
        self.faces = []                        #Lista de caras (no se usa en este código, pero puede usarse para superficies)
        self.edges = []                        #Lista de conexiones entre nodos (aristas)
        self.camera = Node((100, 100, -50))    #Posición inicial de la cámara en el espacio 3D

    #Agrega nodos a la estructura a partir de una lista de coordenadas
    def addNodes(self, nodeList):
        for node in nodeList:
            self.nodes.append(Node(node))

    #Agrega conexiones (aristas) entre nodos (usando pares de índices)
    def addEdges(self, edgeList):
        for edge in edgeList:
            self.edges.append(edge)

    #Transforma el modelo según la vista de cámara (rotaciones y traslación)
    def camera_view(self, heading, pitch, roll):
        #Aplica rotación Z (roll), luego Y (heading), luego X (pitch)
        self.rotateZ(self.findCentre(), roll)
        self.rotateY(self.findCentre(), heading)
        self.rotateX(self.findCentre(), pitch)
        #Traslada el modelo para ajustarlo a la posición de la cámara
        self.translate('x', self.camera.x)
        self.translate('y', self.camera.y)
        self.translate('z', self.camera.z)

    #Desplaza todos los nodos en una dirección (eje) una distancia 'd'
    def translate(self, axis, d):
        """ Añade un desplazamiento 'd' al eje 'axis' de cada nodo del modelo """
        if axis in ['x', 'y', 'z']:
            for node in self.nodes:
                setattr(node, axis, getattr(node, axis) + d)

    #Escala el modelo completo desde un punto central, proporcionalmente en todos los ejes
    def scale(self, centre, scale):
        """ Escala el wireframe desde un punto central (2D) y también el eje Z """
        centre_x, centre_y = centre
        for node in self.nodes:
            node.x = centre_x + scale * (node.x - centre_x)
            node.y = centre_y + scale * (node.y - centre_y)
            node.z *= scale
            #Lo anterior es para el escalado uniforme también en Z

    #Calcula el centro geométrico del modelo (promedio de coordenadas)
    def findCentre(self):
        """ Calcula el centro del modelo (promedio de todos los nodos) """
        num_nodes = len(self.nodes)
        meanX = sum([node.x for node in self.nodes]) / num_nodes
        meanY = sum([node.y for node in self.nodes]) / num_nodes
        meanZ = sum([node.z for node in self.nodes]) / num_nodes

        return (meanX, meanY, meanZ)

    #Rota el modelo completo alrededor del eje X (alrededor del centro proporcionado)
    def rotateX(self, centre, radians):
        cx, cy, cz = centre
        for node in self.nodes:
            y = node.y - cy
            z = node.z - cz
            d = math.hypot(y, z)  #
            #Lo anterior es para la hipotenusa entre y y z
            theta = math.atan2(y, z) + radians
            #Lo anterior es para el angulo actual y la rotación
            node.z = cz + d * math.cos(theta)
            node.y = cy + d * math.sin(theta)

    #Rota el modelo completo alrededor del eje Y
    def rotateY(self, centre, radians):
        cx, cy, cz = centre
        for node in self.nodes:
            x = node.x - cx
            z = node.z - cz
            d = math.hypot(x, z)
            theta = math.atan2(x, z) + radians
            node.z = cz + d * math.cos(theta)
            node.x = cx + d * math.sin(theta)

    #Rota el modelo completo alrededor del eje Z
    def rotateZ(self, centre, radians):
        cx, cy, cz = centre
        for node in self.nodes:
            x = node.x - cx
            y = node.y - cy
            d = math.hypot(y, x)
            theta = math.atan2(y, x) + radians
            node.x = cx + d * math.cos(theta)
            node.y = cy + d * math.sin(theta)
