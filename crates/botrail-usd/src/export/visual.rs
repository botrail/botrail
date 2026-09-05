//! Retain source gprim attributes and referenced material networks. The scene
//! owns placement, visibility and physics; these never leak in from the source.
use super::*;
use openusd::usd::Prim;

pub(super) struct VisualAssets {
    stem: String,
    sources: HashMap<PathBuf, (Stage, String)>,
    pub copies: Vec<(PathBuf, PathBuf)>,
}

impl VisualAssets {
    pub fn new(stem: &str) -> Self {
        Self {
            stem: stem.into(),
            sources: HashMap::new(),
            copies: Vec::new(),
        }
    }

    pub fn author(
        &mut self,
        layer: &mut LayerBuilder,
        dest: &str,
        source: &botrail_model::VisualAsset,
        pose: &XformValue,
        color: Option<[f32; 3]>,
        finish: Option<SurfaceMaterial>,
    ) -> Result<(), UsdExportError> {
        if !self.sources.contains_key(&source.path) {
            let pack = crate::stage_package(&source.path)
                .map_err(|e| UsdExportError::RobotStage(e.to_string()))?;
            let dir = format!(
                "{}_assets/appearances/source_{}",
                self.stem,
                self.sources.len()
            );
            for path in &pack.files {
                self.copies.push((
                    path.clone(),
                    Path::new(&dir).join(path.strip_prefix(&pack.root).unwrap()),
                ));
            }
            let asset = format!(
                "./{dir}/{}",
                pack.stage
                    .strip_prefix(&pack.root)
                    .unwrap()
                    .to_string_lossy()
                    .replace('\\', "/")
            );
            let stage = Stage::builder()
                .resolver(SearchPathResolver::new(vec![pack.root]))
                .open(&pack.stage.to_string_lossy())
                .map_err(|e| UsdExportError::RobotStage(e.to_string()))?;
            self.sources.insert(source.path.clone(), (stage, asset));
        }
        let (stage, asset) = &self.sources[&source.path];
        let prim = stage.prim(sdf::path(&source.prim_path).map_err(author_err)?);
        let type_name = prim.type_name().map_err(author_err)?.ok_or_else(|| {
            UsdExportError::Input(format!("missing visual prim {}", source.prim_path))
        })?;
        layer.ensure_prim(dest, Specifier::Def, Some(type_name.as_str()));
        copy_attributes(layer, dest, &prim)?;
        layer.xform(dest, pose, None);
        layer.attr(
            dest,
            "xformOp:transform:visual",
            "matrix4d",
            AttrValue::Default(Value::Matrix4d(gf::Matrix4d(source.transform))),
        );
        layer.attr(
            dest,
            "xformOpOrder",
            "token[]",
            AttrValue::Uniform(Value::TokenVec(vec![
                "xformOp:translate".into(),
                "xformOp:orient".into(),
                "xformOp:transform:visual".into(),
            ])),
        );
        if let Some(c) = color.filter(|_| {
            source.color_override
                || prim
                    .attribute("primvars:displayColor")
                    .get::<Value>()
                    .ok()
                    .flatten()
                    .is_none()
        }) {
            // A source constant can be inherited above this gprim. Preserve it
            // here; only a scene edit may override a bound shader's colour.
            layer.attr(
                dest,
                "primvars:displayColor",
                "color3f[]",
                AttrValue::Default(Value::Vec3fVec(vec![gf::vec3f(c[0], c[1], c[2])])),
            );
        }
        let tint = source.color_override.then_some(color).flatten();
        bind_material(layer, dest, &prim, asset, tint, finish, &self.copies)?;
        for child in prim.children().map_err(author_err)? {
            if child
                .type_name()
                .map_err(author_err)?
                .is_some_and(|t| t.as_str() == "GeomSubset")
            {
                let name = child.path().as_str().rsplit('/').next().unwrap();
                let sub = format!("{dest}/{name}");
                layer.ensure_prim(&sub, Specifier::Def, Some("GeomSubset"));
                copy_attributes(layer, &sub, &child)?;
                bind_material(layer, &sub, &child, asset, tint, finish, &self.copies)?;
            }
        }
        Ok(())
    }
}

fn author_err(e: impl std::fmt::Display) -> UsdExportError {
    UsdExportError::Author(e.to_string())
}

fn copy_attributes(
    layer: &mut LayerBuilder,
    dest: &str,
    prim: &Prim,
) -> Result<(), UsdExportError> {
    for attr in prim.attributes().map_err(author_err)? {
        let name = attr.path().as_str().split_once('.').unwrap().1;
        if name.starts_with("xformOp")
            || name.starts_with("physics:")
            || name.starts_with("physx")
            || matches!(name, "visibility" | "purpose")
        {
            continue;
        }
        let Some(value) = attr.get::<Value>().map_err(author_err)? else {
            continue;
        };
        let Some(kind) = attr.type_name().map_err(author_err)? else {
            continue;
        };
        let mut meta = Vec::new();
        for key in ["interpolation", "elementSize"] {
            if let Some(value) = attr.get_metadata::<Value>(key).map_err(author_err)? {
                meta.push((key, value));
            }
        }
        layer.attr_meta(dest, name, kind.as_str(), AttrValue::Default(value), &meta);
    }
    Ok(())
}

fn bound_material(prim: &Prim) -> Result<Option<Prim>, UsdExportError> {
    let mut current = Some(prim.path().clone());
    while let Some(path) = current {
        let candidate = prim.stage().prim(path.clone());
        for name in ["material:binding:preview", "material:binding"] {
            let targets = candidate.relationship(name).targets().map_err(author_err)?;
            if let Some(target) = targets.first() {
                return Ok(Some(prim.stage().prim(target.clone())));
            }
        }
        current = path.parent();
    }
    Ok(None)
}

fn bind_material(
    layer: &mut LayerBuilder,
    dest: &str,
    prim: &Prim,
    asset: &str,
    tint: Option<[f32; 3]>,
    finish: Option<SurfaceMaterial>,
    copies: &[(PathBuf, PathBuf)],
) -> Result<(), UsdExportError> {
    let Some(material) = bound_material(prim)? else {
        if let Some(finish) = finish {
            author_surface(layer, dest, finish);
        }
        return Ok(());
    };
    let target = format!("{dest}/BotrailMaterial");
    layer.ensure_prim(&target, Specifier::Def, Some("Material"));
    layer.prim_field(
        &target,
        FieldKey::References,
        Value::ReferenceListOp(ListOp::prepended(vec![Reference {
            asset_path: asset.into(),
            prim_path: material.path().clone(),
            layer_offset: Default::default(),
            custom_data: HashMap::new(),
        }])),
    );
    layer.prim_field(
        dest,
        FieldKey::ApiSchemas,
        Value::TokenListOp(ListOp::prepended(vec!["MaterialBindingAPI".into()])),
    );
    layer.rel(dest, "material:binding", &target);
    if tint.is_some() || finish.is_some() {
        override_surface(layer, &target, &material, tint, finish, copies)?;
    }
    Ok(())
}

fn override_surface(
    layer: &mut LayerBuilder,
    target: &str,
    material: &Prim,
    tint: Option<[f32; 3]>,
    finish: Option<SurfaceMaterial>,
    copies: &[(PathBuf, PathBuf)],
) -> Result<(), UsdExportError> {
    let connections = material
        .attribute("outputs:surface")
        .connections()
        .map_err(author_err)?;
    let Some(path) = connections.first() else {
        return Err(UsdExportError::Input(format!(
            "{}: cannot override material without a surface connection",
            material.path()
        )));
    };
    let shader = material.stage().prim(path.prim_path());
    let id: Option<openusd::tf::Token> = shader.attribute("info:id").get().map_err(author_err)?;
    if id.as_ref().map(|t| t.as_str()) != Some("UsdPreviewSurface") {
        return Err(UsdExportError::Input(format!(
            "{}: finish overrides require UsdPreviewSurface",
            material.path()
        )));
    }
    let prefix = format!("{}/", material.path());
    let rel = shader
        .path()
        .as_str()
        .strip_prefix(&prefix)
        .ok_or_else(|| {
            UsdExportError::Input("surface shader must be inside its material".into())
        })?;
    let dest_shader = format!("{target}/{rel}");
    layer.ensure_prim(&dest_shader, Specifier::Over, None);
    let mut inputs = Vec::new();
    if let Some(c) = tint {
        inputs.push((
            "diffuseColor",
            "color3f",
            Value::Vec3f(gf::vec3f(c[0], c[1], c[2])),
        ));
    }
    if let Some(SurfaceMaterial {
        metalness: m,
        roughness: r,
        opacity,
    }) = finish
    {
        if let Some(a) = opacity {
            inputs.push(("opacity", "float", Value::Float(a)));
        }
        inputs.extend([
            ("metallic", "float", Value::Float(m)),
            ("roughness", "float", Value::Float(r)),
        ]);
    }
    for (name, kind, value) in inputs {
        let attr = shader.attribute(format!("inputs:{name}"));
        if let Some(connection) = attr.connections().map_err(author_err)?.first() {
            let texture = material.stage().prim(connection.prim_path());
            let id: Option<openusd::tf::Token> =
                texture.attribute("info:id").get().map_err(author_err)?;
            if id.as_ref().map(|t| t.as_str()) != Some("UsdUVTexture") {
                return Err(UsdExportError::Input(format!(
                    "{}: overrides require a constant or direct UsdUVTexture input",
                    attr.path()
                )));
            }
            let override_root = format!("{target}/BotrailOverrides");
            let override_texture = format!("{override_root}/{name}");
            layer.ensure_prim(&override_root, Specifier::Def, Some("Scope"));
            layer.ensure_prim(&override_texture, Specifier::Def, Some("Shader"));
            for input in texture.attributes().map_err(author_err)? {
                let prop = input.path().as_str().split_once('.').unwrap().1;
                let Some(kind) = input.type_name().map_err(author_err)? else {
                    continue;
                };
                if let Some(mut v) = input.get::<Value>().map_err(author_err)? {
                    if let Value::AssetPath(a) = &v {
                        let resolved = a.resolved_path().ok_or_else(|| {
                            UsdExportError::Input(format!("unresolved texture {}", a.asset_path()))
                        })?;
                        let (file, suffix) = resolved
                            .split_once('[')
                            .map_or((resolved, String::new()), |(p, entry)| {
                                (p, format!("[{entry}"))
                            });
                        let file = Path::new(file).canonicalize().map_err(author_err)?;
                        let (_, copied) = copies
                            .iter()
                            .find(|(src, _)| src.canonicalize().ok().as_ref() == Some(&file))
                            .ok_or_else(|| {
                                UsdExportError::Input(format!("texture not packaged: {resolved}"))
                            })?;
                        v = Value::AssetPath(
                            format!("./{}{suffix}", copied.to_string_lossy().replace('\\', "/"))
                                .into(),
                        );
                    }
                    layer.attr(
                        &override_texture,
                        prop,
                        kind.as_str(),
                        AttrValue::Default(v),
                    );
                } else {
                    layer.attr(
                        &override_texture,
                        prop,
                        kind.as_str(),
                        AttrValue::Declaration,
                    );
                }
                if let Some(c) = input.connections().map_err(author_err)?.first() {
                    let relative = c.as_str().strip_prefix(&prefix).ok_or_else(|| {
                        UsdExportError::Input(format!(
                            "{}: texture connection leaves its material",
                            c
                        ))
                    })?;
                    layer.connect(
                        &override_texture,
                        prop,
                        kind.as_str(),
                        &format!("{target}/{relative}"),
                    );
                }
            }
            let mut scale: gf::Vec4f = texture
                .attribute("inputs:scale")
                .get()
                .map_err(author_err)?
                .unwrap_or(gf::vec4f(1.0, 1.0, 1.0, 1.0));
            match value {
                Value::Vec3f(c) => {
                    scale.x = c.x;
                    scale.y = c.y;
                    scale.z = c.z;
                }
                Value::Float(v) => match connection.as_str().rsplit(':').next() {
                    Some("g") => scale.y = v,
                    Some("b") => scale.z = v,
                    Some("a") => scale.w = v,
                    _ => scale.x = v,
                },
                _ => unreachable!(),
            }
            layer.attr(
                &override_texture,
                "inputs:scale",
                "float4",
                AttrValue::Default(Value::Vec4f(scale)),
            );
            let output = connection.as_str().split_once('.').unwrap().1;
            layer.connect(
                &dest_shader,
                &format!("inputs:{name}"),
                kind,
                &format!("{override_texture}.{output}"),
            );
            continue;
        }
        layer.attr(
            &dest_shader,
            &format!("inputs:{name}"),
            kind,
            AttrValue::Default(value),
        );
    }
    Ok(())
}
