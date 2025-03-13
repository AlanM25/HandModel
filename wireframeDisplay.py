# Importación de módulos necesarios
import wireframe
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import mediapipe as mp
import cv2
import math

# Puntos clave de la mano
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),  # Pulgar
    (0, 5), (5, 6), (6, 7), (7, 8),  # Índice
    (5, 9), (9, 10), (10, 11), (11, 12),  # Medio
    (9, 13), (13, 14), (14, 15), (15, 16),  # Anular
    (13, 17), (17, 18), (18, 19), (19, 20)  # Meñique
]

# Caras triangulares que forman la superficie visible de la palma (se probó con otras figuras geométricas, pero estas dieron los mejores resultados)
PALM_FACES = [
    (0, 5, 9), (0, 9, 13), (0, 13, 17),
    (5, 9, 13), (9, 13, 17)
]


# Función para dibujar un cilindro entre dos puntos 3D (simula los huesos de los dedos)
def draw_cylinder(x1, y1, z1, x2, y2, z2, radius=10, slices=16):
    vx, vy, vz = x2 - x1, y2 - y1, z2 - z1
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length < 0.0001:
        return
    # Calcula el ángulo de rotación del cilindro para darle movimiento giratorio a la mano
    ax = 57.2957795 * math.acos(vz / length)
    rx, ry, rz = -vy * vz, vx * vz, 0.0

    glPushMatrix()
    glTranslatef(x1, y1, z1)
    glRotatef(ax, rx, ry, rz)
    quadric = gluNewQuadric()
    gluCylinder(quadric, radius, radius * 0.85, length, slices, 1)
    glPopMatrix()


# Función para dibujar una esfera en una posición 3D que simula las articulaciones
def draw_joint(x, y, z, radius=10.0, slices=16, stacks=16):
    glPushMatrix()
    glTranslatef(x, y, z)
    quadric = gluNewQuadric()
    gluSphere(quadric, radius, slices, stacks)
    glPopMatrix()


# Superficie de la palma usando las caras triangulares definidas que se mencionó antes
def draw_palm_surface(hand):
    glColor4f(0.9, 0.7, 0.5, 0.4)  # Color de la palma con transparencia
    glBegin(GL_TRIANGLES)
    for face in PALM_FACES:
        for idx in face:
            node = hand.nodes[idx]
            glVertex3f(node.x, node.y, node.z)
    glEnd()


# Determina el radio del cilindro (grosor del "hueso") según el dedo
def get_finger_radius(i1, i2):
    base_radius = {
        0: 14, 1: 11, 2: 9, 3: 7, 4: 5,
        5: 12, 6: 10, 7: 8, 8: 6,
        9: 12, 10: 10, 11: 8, 12: 6,
        13: 12, 14: 10, 15: 8, 16: 6,
        17: 11, 18: 9, 19: 7, 20: 5
    }
    return min(base_radius.get(i1, 6), base_radius.get(i2, 6))


def get_finger_color(i1, i2):
    if i1 == 0 or i2 == 0:
        return (1.0, 0.85, 0.7)
    return (0.93, 0.75, 0.6)


class ProjectionViewer:
    def __init__(self, width, height):

        self.width = width
        self.height = height
        pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
        pygame.display.set_caption('Hand Model 3D')

        # Configuración de perspectiva y vista 3D
        gluPerspective(45, (width / height), 0.1, 2000.0)
        glTranslatef(0, 0, -750)

        # Habilita iluminación, color y mezcla de transparencia (datos necesarios para que ciertos factores no dañen el resultado)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)
        self.wireframes = {}

    def addWireframe(self, name, wf):
        self.wireframes[name] = wf

    '''Bucle principal del programa: captura cámara, detecta mano y dibuja en 3D'''
    def run(self):
        clock = pygame.time.Clock()
        running = True
        while running:
            clock.tick(30)
            # Lo anterior limita a 30 FPS la visualización

            # Manejo de eventos de cierre de ventana
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False

            # Captura imagen desde la cámara
            ret, frame = self.cap.read()
            if not ret:
                break

            # Convierte imagen a RGB para procesarla con MediaPipe
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.mp_hands.process(frame_rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Dibuja los puntos y conexiones de la mano en el frame
                    self.mp_draw.draw_landmarks(frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)

                    # Actualiza los nodos del wireframe con las posiciones detectadas por MediaPipe
                    for i, lm in enumerate(hand.nodes):
                        if i < 21:
                            landmark = hand_landmarks.landmark[i]
                            lm.x = (landmark.x - 0.5) * 350
                            lm.y = -(landmark.y - 0.5) * 350
                            lm.z = -(landmark.z - 0.1) * 450

            # Muestra el video con los puntos dibujados
            cv2.imshow('MediaPipe Webcam View', frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break  # Salir si se presiona ESC

            # Renderizado del modelo 3D en OpenGL
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            self.displayVolumetric(self.wireframes['Hand'])
            pygame.display.flip()

        # Libera recursos al cerrar
        self.cap.release()
        cv2.destroyAllWindows()
        pygame.quit()

    '''Método para mostrar el modelo 3D (cilindros y esferas)'''
    def displayVolumetric(self, hand):
        # Dibuja superficie de la palma
        draw_palm_surface(hand)

        # Dibuja los cilindros (huesos) entre nodos conectados
        for i1, i2 in hand.edges:
            if i1 < len(hand.nodes) and i2 < len(hand.nodes):
                n1 = hand.nodes[i1]
                n2 = hand.nodes[i2]
                glColor3f(1.0, 1.0, 0.0)
                # Lo anterior le da el color amarillo para huesos
                radius = get_finger_radius(i1, i2)
                draw_cylinder(n1.x, n1.y, n1.z, n2.x, n2.y, n2.z, radius=radius)

        # Dibuja las articulaciones (esferas)
        for idx, node in enumerate(hand.nodes):
            joint_radius = get_finger_radius(idx, idx) * 0.6
            draw_joint(node.x, node.y, node.z, radius=joint_radius)


if __name__ == '__main__':
    from obj_loader import OBJ

    pygame.init()
    pv = ProjectionViewer(800, 600)  # Crea la ventana principal
    obj = OBJ('anatomical_style_hand.obj')  # Carga el modelo OBJ con nodos
    hand = wireframe.Wireframe()  # Crea el wireframe vacío
    hand.addNodes(obj.vertices)  # Añade los vértices del OBJ como nodos
    hand.addEdges(HAND_CONNECTIONS)  # Añade las conexiones (huesos)
    pv.addWireframe('Hand', hand)  # Carga el wireframe al visor
    hand.rotateX(hand.findCentre(), 0.5)  # Rotación inicial
    hand.rotateY(hand.findCentre(), -0.3)
    hand.scale((0, 0), 1.4)
    pv.run()
