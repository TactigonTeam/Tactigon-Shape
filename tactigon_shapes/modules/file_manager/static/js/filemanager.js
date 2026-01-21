class FileManager {
    constructor({ baseUrl, container }) {
        if (!baseUrl) throw new Error("baseUrl is required");
        if (!container) throw new Error("container is required");

        this.baseUrl = baseUrl.endsWith("/") ? baseUrl : baseUrl + "/";
        this.$container = $(container);
        this.current_directory = "";
        this.current_path = [];
        this.current_items = [];
        this.selected_items = [];
    }

    init() {
        this.$container.empty();

        this.buildMenu();
        this.buildSidebar();
        this.buildModals();
        this.buildBody();

        this.$container.append(this.$sidebarContainer, this.$itemsContent);

        this.listDirectory();
        this.updateBody();

        $(window).resize((e) => {
            this._resizeTable();
        });
    }

    buildSidebar() {
        this.$folderList = $('<div>', {
            class: 'list-group list-group-flush flex-grow-1 sidebar',
        });

        this.$sidebarContainer = $('<div>', {
            class: 'd-flex flex-column justify-content-between sidebar-container'
        }).append([
            $('<div>', {
                class: 'text-center pb-3 pt-5 border-bottom'
            }).append(
                $('<h5>').text('Directories')
            ),
            this.$folderList,
            $('<div>', {
                class: 'd-flex flex-column mt-auto p-3 gap-3'
            }).append($('<button>', {
                type: 'button',
                class: 'btn btn-outline-tskin',
                'data-bs-toggle': 'modal',
                'data-bs-target': '#add_directory'
            }).append(
                $('<i>', { class: 'bi bi-plus' }),
                ' Add directory'
            ))
        ]);
    }

    buildModals() {
        this.$addDirectoryModal = $('<div>', {
            class: 'modal fade',
            id: 'add_directory',
            tabindex: -1
        }).append(
            $('<div>', {
                class: 'modal-dialog modal-dialog-centered modal-lg'
            }).append(
                $('<div>', { class: 'modal-content' }).append(
                    $('<form>', { id: 'add_folder_form' }).append([
                        $('<div>', { class: 'modal-header' }).append(
                            $('<h1>', { class: 'modal-title fs-5' }).text('Add directory'),
                            $('<button>', {
                                type: 'button',
                                class: 'btn-close',
                                'data-bs-dismiss': 'modal',
                                'aria-label': 'Close'
                            })),
                        $('<div>', { class: 'modal-body' }).append(
                            $('<div>', { class: 'mb-3' }).append(
                                $('<label>', {
                                    for: 'directory_name',
                                    class: 'form-label'
                                }).text('Name'),
                                $('<input>', {
                                    type: 'text',
                                    class: 'form-control',
                                    id: 'directory_name',
                                    name: 'directory_name'
                                })
                            )
                        ), $('<div>', { class: 'modal-footer' }).append(
                            $('<button>', {
                                type: 'button',
                                class: 'btn btn-outline-secondary',
                                'data-bs-dismiss': 'modal'
                            }).text('Close'),
                            $('<button>', {
                                type: 'submit',
                                class: 'btn btn-primary'
                            }).text('Add folder')
                        )
                    ]).on('submit', (e) => {
                        e.preventDefault();
                        const directoryName = $('#directory_name').val().trim();

                        this.addDirectory(directoryName)
                            .done((res) => {
                                if (!res.success) {
                                    toast(res.error, res.severity);
                                    return;
                                }
                                this.$addDirectoryModal.modal('hide');
                                $('#directory_name').val('');
                                this.listDirectory();
                                this.listDirectoryContent(res.directory.name)
                            })
                            .fail(err => {
                                console.error(err);
                                toast(err, 'danger');
                            });
                    })
                )
            )
        );

        this.$addFolderModal = $('<div>', {
            class: 'modal fade',
            id: 'add_folder',
            tabindex: -1
        }).append(
            $('<div>', {
                class: 'modal-dialog modal-dialog-centered modal-lg'
            }).append(
                $('<div>', { class: 'modal-content' }).append(
                    $('<form>', { id: 'add_folder_form' }).append([
                        $('<div>', { class: 'modal-header' }).append(
                            $('<h1>', { class: 'modal-title fs-5' }).text('Add folder'),
                            $('<button>', {
                                type: 'button',
                                class: 'btn-close',
                                'data-bs-dismiss': 'modal',
                                'aria-label': 'Close'
                            })),
                        $('<div>', { class: 'modal-body' }).append(
                            $('<div>', { class: 'mb-3' }).append(
                                $('<label>', {
                                    for: 'folder_name',
                                    class: 'form-label'
                                }).text('Name'),
                                $('<input>', {
                                    type: 'text',
                                    class: 'form-control',
                                    id: 'folder_name',
                                    name: 'folder_name'
                                })
                            )
                        ), $('<div>', { class: 'modal-footer' }).append(
                            $('<button>', {
                                type: 'button',
                                class: 'btn btn-outline-secondary',
                                'data-bs-dismiss': 'modal'
                            }).text('Close'),
                            $('<button>', {
                                type: 'submit',
                                class: 'btn btn-primary'
                            }).text('Add folder')
                        )
                    ]).on('submit', (e) => {
                        e.preventDefault();
                        const folderPath = this.current_path.slice(1).join("/");
                        const folderName = $('#folder_name').val().trim();

                        this.addFolder(this.current_directory, folderPath, folderName)
                            .done((res) => {
                                if (!res.success) {
                                    toast(res.error, res.severity);
                                    return;
                                }
                                this.$addFolderModal.modal('hide');
                                $('#folder_name').val('');
                                this.listFolderContent();
                                toast("Folder created!", "success")
                            })
                            .fail(err => {
                                console.error(err);
                                toast(err.responseJSON.error, 'danger');
                            });
                    })
                )
            )
        );

        this.$addFileModal = $('<div>', {
            class: 'modal fade',
            id: 'add_file',
            tabindex: -1
        }).append(
            $('<div>', {
                class: 'modal-dialog modal-dialog-centered modal-lg'
            }).append(
                $('<div>', { class: 'modal-content' }).append(
                    $('<form>', { id: 'add_folder_form' }).append([
                        $('<div>', { class: 'modal-header' }).append(
                            $('<h1>', { class: 'modal-title fs-5' }).text('Add file'),
                            $('<button>', {
                                type: 'button',
                                class: 'btn-close',
                                'data-bs-dismiss': 'modal',
                                'aria-label': 'Close'
                            })),
                        $('<div>', { class: 'modal-body' }).append(
                            $('<div>', { class: 'mb-3' }).append(
                                $('<label>', {
                                    for: 'file_input',
                                    class: 'form-label'
                                }).text('File'),
                                $('<input>', {
                                    type: 'file',
                                    class: 'form-control',
                                    id: 'file_input',
                                    name: 'file_input'
                                })
                            )
                        ), $('<div>', { class: 'modal-footer' }).append(
                            $('<button>', {
                                type: 'button',
                                class: 'btn btn-outline-secondary',
                                'data-bs-dismiss': 'modal'
                            }).text('Close'),
                            $('<button>', {
                                type: 'submit',
                                class: 'btn btn-primary'
                            }).text('Add file')
                        )
                    ]).on('submit', (e) => {
                        e.preventDefault();

                        const folder_path = this.current_path.slice(1).join("/");
                        const file_input = $('#file_input')[0];

                        if (!file_input.files.length) {
                            toast('Please select a file', "danger");
                            return;
                        }

                        this.uploadFile(this.current_directory, folder_path, file_input.files[0])
                            .done((res) => {
                                if (!res.success) {
                                    toast(res.error, res.severity);
                                    return;
                                }
                                this.$addFileModal.modal('hide');
                                $('#file_input').val('');
                                this.listFolderContent();
                                toast("File uploaded!", "success")
                            })
                            .fail(err => {
                                console.error(err);
                                toast(err.responseJSON.error, 'danger');
                            });
                    })
                )
            )
        );

        this.$container.append(this.$addDirectoryModal, this.$addFolderModal, this.$addFileModal);
    }

    buildBody() {
        this.$currentFolderContent = $("<tbody>");
        this.$emptyFolderContent = $("<div>", { class: "d-flex flex-column justify-content-center align-items-center gap-1 p-3" }).append(
            $("<i>", { class: "mi-outlined mi-folder text-muted" }).css("fontSize", "48px"),
            $("<span>").text("Folder is empty")
        );
        this.$folderContentContainer = $("<div>", { class: "table-wrapper" })
            .append(
                $("<table>", { class: "table table-sm table-hover table-borderless" })
                    .append(
                        $("<thead>").append(
                            $("<tr>").append(
                                $("<th>").text("Name"),
                                $("<th>", { class: "fixed-100px" }).text("Size"),
                                $("<th>", { class: "fixed-100px" }).text("Modified"),
                            )
                        ),
                        this.$currentFolderContent
                    )
            );

        this.$breadcrumb = $("<ol>", { class: "breadcrumb my-auto" });
        this.$backButton = $("<button>", { type: "button", class: "btn btn-link text-secondary" })
            .html(`<i class="mi-outlined mi-arrow-back-ios-new"></i>`)
            .on("click", (e) => {
                e.preventDefault();
                if (this.current_path.length > 1) {
                    e.preventDefault();
                    this.current_path = this.current_path.slice(0, -1);
                    this.listFolderContent();
                }
            });

        this.$downloadItemsButton = $("<button>", {
            type: "button",
            class: "btn btn-outline-secondary btn-sm",
            title: "Download file"
        }).append(
            $("<i>", { class: "mi-outlined mi-file-download" })
        ).click((e) => {
            e.preventDefault();
            this.downloadItems(this.current_directory, this.selected_items)
                .done((blob, status, xhr) => {
                    this._saveFile(blob, status, xhr);
                    this.selected_items = [];
                    this.updateBody();
                })
                .fail(err => {
                    console.error(err);
                    toast(err.responseJSON.error, 'danger');
                });
        });

        this.$deleteItemsButton = $("<button>", {
            type: "button",
            class: "btn btn-outline-secondary text-danger btn-sm",
            title: "Delete items",
            // 'data-bs-toggle': 'modal',
            // 'data-bs-target': '#add_folder'
        }).append(
            $("<i>", { class: "mi-outlined mi-delete" })
        ).click((e) => {
            e.preventDefault();
            this.deleteItems(this.current_directory, this.selected_items)
                .done((res) => {
                    if (!res.success) {
                        toast(res.error, res.severity);
                        return;
                    }

                    this.listFolderContent();
                    toast("Items removed!", "success")
                })
                .fail(err => {
                    console.error(err);
                    toast(err.responseJSON.error, 'danger');
                });
        });

        this.$itemsBodyContent = $("<div>", {
            class: "card-body p-1 h-100"
        }).append(
            $("<div>", { class: "d-flex justify-content-between", id: "navigation" }).append(
                this.$backButton,
                $("<nav>", { class: "breadcrumb m-0" }).append(
                    this.$breadcrumb
                ),
                $("<div>") // Placeholder for alignment
            ),
            this.$folderContentContainer,
            this.$emptyFolderContent
        );

        this.$itemsContent = $('<div>', {
            class: 'd-flex flex-column flex-fill p-3 gap-3',
        })
            // .css("max-height", "600px")
            .append(
                $("<div>", { class: "btn-toolbar gap-3", id: "toolbar" }).append(
                    $("<div>", { class: "btn-group" }).append(
                        $("<button>", {
                            type: "button",
                            class: "btn btn-outline-secondary btn-sm",
                            title: "Refresh"
                        }).append(
                            $("<i>", { class: "mi-outlined mi-refresh" })
                        ).on("click", (e) => {
                            e.preventDefault();
                            this.listFolderContent();
                            toast("Content refreshed!", "success")
                        }),
                        $("<button>", {
                            type: "button",
                            class: "btn btn-outline-secondary btn-sm",
                            title: "Create directory",
                            'data-bs-toggle': 'modal',
                            'data-bs-target': '#add_folder'
                        }).append(
                            $("<i>", { class: "mi-outlined mi-create-new-folder" })
                        )
                    ),
                    $("<div>", { class: "btn-group" }).append(
                        $("<button>", {
                            type: "button",
                            class: "btn btn-outline-secondary btn-sm",
                            title: "Upload file",
                            'data-bs-toggle': 'modal',
                            'data-bs-target': '#add_file'
                        }).append(
                            $("<i>", { class: "mi-outlined mi-file-upload" })
                        ),
                        this.$downloadItemsButton,
                        this.$deleteItemsButton
                    )
                ),
                $("<div>", {
                    class: "d-flex flex-fill card"
                }).append(
                    this.$itemsBodyContent
                )
            );
    }

    buildMenu() {
        this.$contextMenu = $('<div>', {
            id: 'context_menu',
            class: 'dropdown-menu shadow d-none'
        })
            .css({
                position: 'absolute',
                zIndex: 1055
            })
            .append(
                $('<button>', {
                    class: 'dropdown-item',
                    'data-action': 'download'
                }).append(
                    $('<i>', { class: 'mi-outlined mi-file-download me-2' }),
                    'Download'
                ),
                $('<button>', {
                    class: 'dropdown-item',
                    'data-action': 'select'
                }).append(
                    $('<i>', { class: 'mi-outlined mi-check-box me-2' }),
                    'Select'
                ),
                $('<button>', {
                    class: 'dropdown-item',
                    'data-action': 'delete'
                }).append(
                    $('<i>', { class: 'mi-outlined mi-delete me-2' }),
                    'Delete'
                )
            )
            .on('click', '.dropdown-item', (e) => {
                e.preventDefault();

                const el = $(e.currentTarget);
                const action = el.data("action");
                const item = this.$contextMenu.data("item");

                switch (action) {
                    case 'download':
                        this.downloadItems(this.current_directory, [item])
                            .done(this._saveFile)
                            .fail(err => {
                                console.error(err);
                                toast(err.responseJSON.error, 'danger');
                            });
                        break;
                    case 'select':
                        this.selected_items.push(item)
                        break;
                    case 'delete':
                        this.deleteItems(this.current_directory, [item])
                            .done((res) => {
                                if (!res.success) {
                                    toast(res.error, res.severity);
                                    return;
                                }

                                this.current_items = this.current_items.filter((e) => e !== item);
                                toast("File removed!", "success");
                                this.updateBody();
                            })
                            .fail(err => {
                                console.error(err);
                                toast(err.responseJSON.error, 'danger');
                            });
                        break;
                }

                this.$contextMenu.hide();
                this.updateBody();
            });

        this.$container.append(this.$contextMenu);

        $(document).on('click.contextMenu', (e) => {
            if (!this.$contextMenu.is(e.target) && this.$contextMenu.has(e.target).length === 0) {
                this.$contextMenu.hide();
            }
        });
    }

    updateBody() {
        this.$currentFolderContent.empty();
        this.$breadcrumb.empty();

        if (this.current_directory == "") {
            this.$itemsContent.addClass("d-none");
            return;
        }

        this.$itemsContent.removeClass("d-none");

        this.$breadcrumb.append(
            $.map(this.current_path, (part, index) => {
                const isLast = index === this.current_path.length - 1;
                return $("<li>", { class: `breadcrumb-item${isLast ? ' active' : ''}`, 'aria-current': isLast ? 'page' : null }).append(
                    isLast ? part : $("<a>", { href: "#", text: part }).on("click", (e) => {
                        e.preventDefault();
                        this.current_path = this.current_path.slice(0, index + 1);
                        this.listFolderContent();
                    })
                );
            })
        )

        if (this.current_path.length > 1) {
            this.$backButton.removeClass("disabled");
        } else {
            this.$backButton.addClass("disabled");
        }

        if (this.current_items.length == 0) {
            this.$folderContentContainer.addClass("d-none");
            this.$emptyFolderContent.removeClass("d-none");
            return;
        }

        this.$currentFolderContent.append(
            $.map(this.current_items, (item) => {
                var selection_checkbox = "";
                if (this.selected_items.length > 0) {
                    const checked = this.selected_items.indexOf(item) > -1;

                    selection_checkbox = $("<input>", {
                        type: "checkbox",
                        class: "form-check-input",
                        id: item.name
                    }).change((e) => {
                        const checkbox = $(e.currentTarget);
                        const checked = checkbox.prop("checked");

                        if (!checked) {
                            this.selected_items = this.selected_items.filter(i => i !== item);
                        } else {
                            this.selected_items.push(item);
                        }

                        this.updateBody();
                    })
                    selection_checkbox.prop("checked", checked);
                }

                return $("<tr>")
                    .css("cursor", "pointer")
                    .append($("<td>")
                        .append(
                            $("<div>", { class: "form-check" }).append(
                                selection_checkbox,
                                $("<label>", { class: "form-check-label", for: item.name }).append(
                                    `<i class="mi-outlined mi-${item.item_type === 'folder' ? 'folder' : 'file-open'} me-2"></i>`,
                                    item.name
                                )
                            )
                        ),
                        $("<td>", { class: "fixed-100px" }).text(item.size ? this._getFileSize(item.size) : ""),
                        $("<td>", { class: "fixed-100px" }).text(item.modified_time ? new Date(item.modified_time).toLocaleDateString() : ""),
                    )
                    .on("contextmenu", (e) => {
                        e.preventDefault();
                        this.showContextMenu(e, item);
                    })
                    .on("dblclick", (e) => {
                        e.preventDefault();
                        if (item.item_type === 'folder') {
                            this.listFolderContent(item.name);
                        }
                    })
            })
        );

        if (this.selected_items.length > 0) {
            this.$deleteItemsButton.removeClass("disabled");
            this.$downloadItemsButton.removeClass("disabled");
        } else {
            this.$deleteItemsButton.addClass("disabled");
            this.$downloadItemsButton.addClass("disabled");
        }

        this.$folderContentContainer.removeClass("d-none");
        this.$emptyFolderContent.addClass("d-none");

        this._resizeTable();
    }

    showContextMenu(e, item) {
        e.preventDefault(); // evita menu nativo
        let left = e.pageX;
        let top = e.pageY;

        this.$contextMenu
            .data("item", item)
            .css({ top, left })
            .show() // basta .show() di jQuery
            .removeClass('d-none'); // se vuoi compatibilità classi
    }

    sortItems(items) {
        this.current_items = items;
        this.current_items.sort((a, b) => {
            if (a.item_type === b.item_type) {
                return a.name.localeCompare(b.name);
            }
            return a.item_type === 'folder' ? -1 : 1;
        });
    }

    listDirectory() {
        this._ajax({
            url: "directories/",
            method: "GET",
        })
            .done((response) => {
                this.$folderList.empty();
                response.directories.forEach((folder) => {
                    const $folderItem = $('<a>', {
                        href: '#',
                        class: 'list-group-item list-group-item-action' + (folder.name === this.current_directory ? ' active' : ''),
                        html: [$("<i>", { class: 'mi-outlined mi-folder me-2' }), folder.name],
                    }).click((e) => {
                        e.preventDefault();
                        this.$folderList.children().removeClass('active');
                        $(e.currentTarget).addClass('active');
                        this.listDirectoryContent(folder.name)
                    });

                    this.$folderList.append($folderItem);
                });
            })
            .fail(err => {
                console.error(err);
                toast(err.responseJSON.error, 'danger');
            });
    }

    addDirectory(name) {
        return this._ajax({
            url: "directories/add",
            method: "POST",
            data: {
                directory_name: name,
            },
        });
    }

    addFolder(directory, folder_path, folder_name) {
        return this._ajax({
            url: `directories/${encodeURIComponent(directory)}/folders/add`,
            method: "POST",
            data: {
                folder_path: folder_path,
                folder_name: folder_name,
            },
        });
    }

    listDirectoryContent(directoryName) {
        this._ajax({
            url: `directories/${encodeURIComponent(directoryName)}/content`,
            method: "GET",
        }).done((res) => {
            this.current_directory = directoryName;
            this.current_path = [];
            this.current_path.push(directoryName);
            this.selected_items = []
            this.sortItems(res.items);
            this.updateBody();
        })
            .fail(err => {
                console.error(err);
                toast(err.responseJSON.error, 'danger');
            });
    }

    listFolderContent(folderName) {
        let subfolders = [...this.current_path].splice(1).join(",");

        if (subfolders != "") {
            subfolders += ",";
        }

        if (folderName != undefined && folderName != "") {
            subfolders += folderName;
        }

        this._ajax({
            url: `directories/${encodeURIComponent(this.current_directory)}/content`,
            method: "GET",
            data: {
                subfolders: subfolders,
            }
        }).done((res) => {
            if (folderName != undefined && folderName != "") {
                this.current_path.push(folderName);
            }
            this.selected_items = []
            this.sortItems(res.items);
            this.updateBody();
        })
            .fail(err => {
                console.error(err);
                toast(err.responseJSON.error, 'danger');
            });
    }

    uploadFile(directoryName, folder_path, file) {
        const formData = new FormData();
        formData.append('file', file);
        formData.append('folder_path', folder_path);

        return $.ajax({
            url: `${this.baseUrl}directories/${encodeURIComponent(directoryName)}/files/add`,
            method: 'POST',
            data: formData,
            processData: false,
            contentType: false,
            dataType: 'json'
        }).then(res => {
            if (!res.success) {
                return $.Deferred().reject(res);
            }
            return res;
        });
    }

    deleteItems(directoryName, items) {
        return $.ajax({
            url: `${this.baseUrl}directories/${encodeURIComponent(directoryName)}/files/delete`,
            method: 'DELETE',
            contentType: 'application/json',
            data: JSON.stringify({ items: items }),
            dataType: 'json'
        }).then(res => {
            if (!res.success) {
                return $.Deferred().reject(res);
            }
            return res;
        });
    }

    downloadItems(directoryName, items) {
        return $.ajax({
            url: `${this.baseUrl}directories/${encodeURIComponent(directoryName)}/files/download`,                         // es: '/api/file-manager/.../download'
            method: 'POST',
            contentType: 'application/json',
            data: JSON.stringify({ items }),
            xhrFields: {
                responseType: 'blob'      // ricevi blob (ZIP)
            },
        });
    }

    _ajax({ url, method = "GET", data = null }) {
        return $.ajax({
            url: this.baseUrl + url,
            method,
            data,
            dataType: "json",
        }).then((response) => {
            if (!response.success) {
                return $.Deferred().reject(response);
            }
            return response;
        });
    }

    _saveFile(blob, status, xhr) {
        let filename = `download.zip`;
        const disposition = xhr.getResponseHeader("Content-Disposition");
        if (disposition && disposition.indexOf("attachment") !== -1) {
            const filenameRegex = /filename[^;=\n]*=((['"]).*?\2|[^;\n]*)/;
            const matches = filenameRegex.exec(disposition);
            if (matches != null && matches[1]) {
                filename = matches[1].replace(/['"]/g, '');
            }
        }

        const downloadUrl = window.URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = downloadUrl;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        a.remove();
        window.URL.revokeObjectURL(downloadUrl);
    }

    _resizeTable() {
        const itemsContentHeight = this.$itemsContent.innerHeight();
        const navigationHeight = this.$itemsBodyContent.find("#navigation").outerHeight();
        const toolbarHeight = this.$itemsContent.find("#toolbar").outerHeight();

        const table = itemsContentHeight - (2 * 4) - (16 * 3) - navigationHeight - toolbarHeight; // deve essere circa 730
        this.$folderContentContainer.height(`${table}px`);
    }

    _getFileSize(size) {
        if (size > 1000000) {
            return `${round(size / 1000000, 2)} GB`
        }

        if (size > 1000) {
            return `${round(size / 1000, 2)} MB`
        }

        if (size > 100) {
            return `${round(size / 100, 2)} kB`
        }

        return `${round(size, 2)} bytes`
    }
}
