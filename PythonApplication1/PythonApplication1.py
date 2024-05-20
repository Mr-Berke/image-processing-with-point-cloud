from PyQt5 import QtCore, QtGui, QtWidgets
import open3d as o3d
import numpy as np

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(400, 300)
        self.lineEdit = QtWidgets.QLineEdit(Form)
        self.lineEdit.setGeometry(QtCore.QRect(40, 40, 200, 31))
        self.lineEdit.setObjectName("lineEdit")
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(260, 40, 93, 28))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(Form)
        self.pushButton_2.setGeometry(QtCore.QRect(50, 170, 131, 28))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(Form)
        self.pushButton_3.setGeometry(QtCore.QRect(212, 170, 131, 28))
        self.pushButton_3.setObjectName("pushButton_3")
        self.pushButton_4 = QtWidgets.QPushButton(Form)
        self.pushButton_4.setGeometry(QtCore.QRect(50, 210, 131, 28))
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_5 = QtWidgets.QPushButton(Form)
        self.pushButton_5.setGeometry(QtCore.QRect(212, 210, 131, 28))
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_6 = QtWidgets.QPushButton(Form)
        self.pushButton_6.setGeometry(QtCore.QRect(50, 250, 293, 28))
        self.pushButton_6.setObjectName("pushButton_6")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)
        
        # Butonlar�n fonksiyonlara ba�lanmas�
        self.pushButton.clicked.connect(self.chooseFile)
        self.pushButton_2.clicked.connect(self.openFile)
        self.pushButton_3.clicked.connect(self.findPlanes)
        self.pushButton_4.clicked.connect(self.findMultiplePlanes)
        self.pushButton_5.clicked.connect(self.findPointsAboveThreshold)
        self.pushButton_6.clicked.connect(self.findConvexHullWithNormals)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.pushButton.setText(_translate("Form", "Dosya Sec"))
        self.pushButton_2.setText(_translate("Form", "Dosya Ac"))
        self.pushButton_3.setText(_translate("Form", "Duzlem Bul"))
        self.pushButton_4.setText(_translate("Form", "Duzlemleri Bul"))
        self.pushButton_5.setText(_translate("Form", "100 nokta ustu bul"))
        self.pushButton_6.setText(_translate("Form", "Convex Hull ve Normalleri Bul"))

    def chooseFile(self):
        # Dosya se�me i�lemi
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Dosya Sec", "", "All Files (*)")
        if fileName:
            self.lineEdit.setText(fileName)

    def openFile(self):
        # Dosya a�ma i�lemi
        file_path = self.lineEdit.text()
        if file_path:
            # PLY dosyas�n�n y�klenmesi
            point_cloud = o3d.io.read_point_cloud(file_path)
            # Nokta bulutunun g�rselle�tirilmesi
            o3d.visualization.draw_geometries([point_cloud])

    def findPlanes(self):
        # Dosya yolu alma i�lemi
        file_path = self.lineEdit.text()
        if file_path:
            # PLY dosyas�n�n y�klenmesi
            point_cloud = o3d.io.read_point_cloud(file_path)
            # RANSAC algoritmas� kullan�larak d�zlemlerin bulunmas�
            plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.05,
                                                             ransac_n=4,
                                                             num_iterations=5000)
            # D�zlem ve i�erisindeki noktalar�n al�nmas�
            planes = [plane_model]
            inlier_clouds = [point_cloud.select_by_index(inliers)]
            # Sonu�lar�n g�rselle�tirilmesi
            o3d.visualization.draw_geometries(inlier_clouds)

    def findMultiplePlanes(self):
        # Dosya yolu alma i�lemi
        file_path = self.lineEdit.text()
        if file_path:
            # PLY dosyas�n�n y�klenmesi
            point_cloud = o3d.io.read_point_cloud(file_path)
            num_planes = 4  # Bulunacak d�zlem say�s�
            planes = []
            inlier_clouds = []
            for i in range(num_planes):
                # D�zlemin bulunmas�
                plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.05,
                                                                  ransac_n=4,
                                                                  num_iterations=5000)
                # Kalan i�erisindeki noktalar�n al�nmas�
                planes.append(plane_model)
                # ��erisindeki inlier noktalar�n se�ilmesi
                inlier_cloud = point_cloud.select_by_index(inliers)
                # Rastgele bir renk atanmas�
                inlier_cloud.paint_uniform_color([np.random.rand(), np.random.rand(), np.random.rand()])
                # Inlier bulutunun listeye eklenmesi
                inlier_clouds.append(inlier_cloud)
                # Bulunan d�zlemin i�erisindeki noktalar�n orijinal nokta buluttan ��kar�lmas�
                point_cloud = point_cloud.select_by_index(inliers, invert=True)
            # Sonu�lar�n g�rselle�tirilmesi
            o3d.visualization.draw_geometries(inlier_clouds)

    def findPointsAboveThreshold(self):
        # Dosya yolu alma i�lemi
        file_path = self.lineEdit.text()
        if file_path:
            # PLY dosyas�n�n y�klenmesi
            point_cloud = o3d.io.read_point_cloud(file_path)

            def find_planes(point_cloud, min_points_per_plane=100, distance_threshold=0.05, ransac_n=4, num_iterations=5000):
                planes = []
                inlier_clouds = []

                while len(point_cloud.points) >= min_points_per_plane:
                    # D�zlemin bulunmas�
                    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                                      ransac_n=ransac_n,
                                                                      num_iterations=num_iterations)
                    # ��erisindeki inlier noktalar�n se�ilmesi
                    inlier_cloud = point_cloud.select_by_index(inliers)

                    # Rastgele bir renk atanmas�
                    inlier_cloud.paint_uniform_color([np.random.rand(), np.random.rand(), np.random.rand()])

                    # Inlier bulutunun listeye eklenmesi
                    inlier_clouds.append(inlier_cloud)

                    # Bulunan d�zlemin i�erisindeki noktalar�n orijinal nokta buluttan ��kar�lmas�
                    point_cloud = point_cloud.select_by_index(inliers, invert=True)

                    # E�er bulunan d�zlemin i�erisindeki nokta say�s� e�ik de�erden k���kse d�ng� sonlan�r
                    if len(inliers) < min_points_per_plane:
                        break

                    # Bulunan d�zlemlerin kaydedilmesi
                    planes.append(plane_model)

                return planes, inlier_clouds

            # En az 100 noktas� olan d�zlemlerin bulunmas�
            planes, result = find_planes(point_cloud, min_points_per_plane=100)

            # Sonu�lar�n g�rselle�tirilmesi
            o3d.visualization.draw_geometries(result)

    def findConvexHullWithNormals(self):
        # Dosya yolu alma i�lemi
        file_path = self.lineEdit.text()
        if file_path:
            # PLY dosyas�n�n y�klenmesi
            point_cloud = o3d.io.read_point_cloud(file_path)

            def find_planes_and_hulls(point_cloud, min_points_per_plane=10000, distance_threshold=0.05, ransac_n=4, num_iterations=5000):
                planes = []
                hulls = []
                normal_lines = []

                while len(point_cloud.points) >= min_points_per_plane:
                    # D�zlemin bulunmas�
                    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                                      ransac_n=ransac_n,
                                                                      num_iterations=num_iterations)
                    # ��erisindeki inlier noktalar�n se�ilmesi
                    inlier_cloud = point_cloud.select_by_index(inliers)

                    # Convex hull hesaplama
                    hull, _ = inlier_cloud.compute_convex_hull()
                    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
                    hull_ls.paint_uniform_color([np.random.rand(), np.random.rand(), np.random.rand()])

                    # Convex hull'un normal vekt�rlerini hesaplama ve g�rselle�tirme
                    vertices = np.asarray(hull.vertices)
                    triangles = np.asarray(hull.triangles)

                    for triangle in triangles:
                        v0 = vertices[triangle[0]]
                        v1 = vertices[triangle[1]]
                        v2 = vertices[triangle[2]]
                        center = (v0 + v1 + v2) / 3
                        normal = np.cross(v1 - v0, v2 - v0)
                        normal = normal / np.linalg.norm(normal)
                        
                        # Normal vekt�r�n� g�rselle�tirme
                        line_set = o3d.geometry.LineSet(
                            points=o3d.utility.Vector3dVector([center, center + normal * 0.1]),
                            lines=o3d.utility.Vector2iVector([[0, 1]]),
                        )
                        line_set.paint_uniform_color([1, 0, 0])  # Normal vekt�rler k�rm�z� renkte
                        normal_lines.append(line_set)

                    # Convex hull'u listeye ekleme
                    hulls.append(hull_ls)

                    # Bulunan d�zlemin i�erisindeki noktalar�n orijinal nokta buluttan ��kar�lmas�
                    point_cloud = point_cloud.select_by_index(inliers, invert=True)

                    # E�er bulunan d�zlemin i�erisindeki nokta say�s� e�ik de�erden k���kse d�ng� sonlan�r
                    if len(inliers) < min_points_per_plane:
                        break

                    # Bulunan d�zlemlerin kaydedilmesi
                    planes.append(plane_model)

                return planes, hulls, normal_lines

            # En az 100 noktas� olan d�zlemler ve convex hull'lar�n�n bulunmas�
            planes, result_hulls, result_normals = find_planes_and_hulls(point_cloud, min_points_per_plane=100)

            # Sonu�lar�n g�rselle�tirilmesi (sadece convex hull'lar ve normal vekt�rler)
            o3d.visualization.draw_geometries(result_hulls + result_normals)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
