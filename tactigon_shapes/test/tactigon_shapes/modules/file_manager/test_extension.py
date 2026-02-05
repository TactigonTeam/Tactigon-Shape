from unittest import TestCase
from unittest.mock import patch, MagicMock

from tactigon_shapes.modules.file_manager.extension import FileManager
from tactigon_shapes.modules.file_manager.models import DirectoryItem, FolderItem, FileItem

class TestFileManager(TestCase):

    @patch("tactigon_shapes.modules.file_manager.extension.os.path.exists")
    @patch("tactigon_shapes.modules.file_manager.extension.os.walk")
    def test_list_folders_directory_exists(self, mock_walk, mock_exists):
        # Arrange
        mock_exists.return_value = True
        mock_walk.return_value = [
            ("/base/path", [], []),
            ("/base/path/sub1", [], []),
            ("/base/path/sub1/__pycache__", [], []),
        ]

        directory = DirectoryItem(base_path="/base/path", name="directory")

        result = FileManager.list_folders(directory)

        self.assertEqual(len(result), 2)
        mock_exists.assert_called_once_with("/base/path")

    @patch("tactigon_shapes.modules.file_manager.extension.os.path.exists")
    def test_list_folders_directory_not_exists(self, mock_exists):
        # Arrange
        mock_exists.return_value = False
        directory = DirectoryItem(base_path="/base/path", name="directory")

        # Act
        result = FileManager.list_folders(directory)

        # Assert
        self.assertEqual(result, [])
        mock_exists.assert_called_once_with("/base/path")



    @patch("tactigon_shapes.modules.file_manager.extension.os.scandir")
    @patch("tactigon_shapes.modules.file_manager.extension.os.path.isdir")
    @patch("tactigon_shapes.modules.file_manager.extension.os.path.exists")
    def test_list_files_directory_exists(
        self, mock_exists, mock_isdir, mock_scandir
    ):
        # Arrange
        mock_exists.return_value = True
        mock_isdir.return_value = True

        # Creiamo un file finto
        fake_entry = MagicMock()
        fake_entry.is_file.return_value = True
        fake_entry.name = "test.txt"
        fake_entry.path = "/base/path/test.txt"
        fake_entry.stat.return_value.st_size = 123

        mock_scandir.return_value = [fake_entry]

        directory = DirectoryItem(base_path="/base/path", name="directory")
        result = FileManager.list_files(directory)

        self.assertEqual(len(result), 1)

        mock_exists.assert_called_once_with("/base/path")
        mock_isdir.assert_called_once_with("/base/path")
        mock_scandir.assert_called_once_with("/base/path")

    @patch("tactigon_shapes.modules.file_manager.extension.os.path.isdir")
    @patch("tactigon_shapes.modules.file_manager.extension.os.path.exists")
    def test_list_files_directory_not_exists(
        self, mock_exists, mock_isdir
    ):
        mock_exists.return_value = False
        mock_isdir.return_value = False
        directory = DirectoryItem(base_path="/base/path", name="directory")

        result = FileManager.list_files(directory)

        self.assertEqual(result, [])

    @patch("tactigon_shapes.modules.file_manager.extension.os.scandir")
    @patch("tactigon_shapes.modules.file_manager.extension.os.path.isdir")
    @patch("tactigon_shapes.modules.file_manager.extension.os.path.exists")
    def test_list_files_with_folder(
        self, mock_exists, mock_isdir, mock_scandir
    ):
        # Arrange
        mock_exists.return_value = True
        mock_isdir.return_value = True
        mock_scandir.return_value = []

        directory = DirectoryItem(base_path="/base", name="dir")
        folder = FolderItem(base_path="sub", name="folder")

        # Act
        result = FileManager.list_files(directory, folder)

        # Assert
        self.assertEqual(result, [])

    def test_get_file_extension(self):

        for filename, expected in [("my-filename", ""), ("my-file.png", ".png"), ("file.tar.gz", ".tar.gz")]:
            result = FileManager.get_file_extension(filename)
            self.assertEqual(result, expected)
