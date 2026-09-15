#include "RotationBodyPlanningTranslations.h"

#include <array>

namespace smrobot::workbench::spray::rotationbody
{
    namespace
    {
        struct TranslationEntry
        {
            const char* key;
            const char* english;
            const char* chinese;
        };

        constexpr TranslationEntry entries[] = {
            { "status.restored_source_changed", "The source model changed. The model was realigned; confirm the workpiece frame and rebuild the section and regions.", "源模型已变更，系统已重新摆正。请重新确认工件坐标系，并重新生成剖面和区域。" },
            { "error.viewport_presentation", "The model view or planning overlay could not be restored. Reload the view or re-enter this module.", "无法恢复模型视图或规划叠加层。请重新加载视图或重新进入本模块。" },
            { "workflow.title", "Rotation-body workflow", "回转体规划工作流" },
            { "workflow.model_transform", "Model Transform", "模型变换" },
            { "workflow.section_region", "Section Partition", "剖切分区" },
            { "right_workflow.trajectory", "Trajectory Planning & Adjustment", "轨迹规划与调整" },
            { "right_workflow.abb", "ABB Instruction Translation", "ABB 指令转译" },
            { "right_workflow.calibration", "Workpiece Calibration", "工件标定" },
            { "right_workflow.trajectory_short", "Trajectory", "轨迹规划" },
            { "right_workflow.abb_short", "ABB", "ABB 转译" },
            { "right_workflow.calibration_short", "Calibration", "工件标定" },
            { "calibration.title", "Workpiece Frame Calibration", "工件坐标系标定" },
            { "calibration.intro", "Enter robot-base touch coordinates in millimeters. The calculated pose is applied directly to the base-coordinate workpiece pose on the left.", "输入机器人基坐标系下的触碰点坐标（毫米）。计算结果会直接回填左侧基坐标系下的工件位姿。" },
            { "calibration.mode_one", "Mode 1", "模式一" },
            { "calibration.mode_two", "Mode 2", "模式二" },
            { "calibration.mode_cylinder", "Spatial cylinder", "空间圆柱" },
            { "calibration.mode_circle", "Horizontal circle", "水平圆" },
            { "calibration.cylinder_hint", "Touch 8-12 points on the cylindrical side at multiple heights. The points do not need to be coplanar.", "在圆柱侧面不同高度触碰 8~12 个点，触碰点无需共面。" },
            { "calibration.circle_hint", "Touch 6 points on one horizontal circle. The maximum Z difference must not exceed 0.1 mm.", "在同一水平圆上触碰 6 个点，最大 Z 差不得超过 0.1 mm。" },
            { "calibration.import_mode_two", "Read Mode 2 TXT", "读取模式二 TXT" },
            { "calibration.import_mode_two_tooltip", "Read 6 circle points, 3 reference points and 1 ABB safety point from a slash-separated TXT file.", "从斜杠分隔的 TXT 文件读取 6 个圆拟合点、3 个参考点和 1 个 ABB 安全点。" },
            { "calibration.import_dialog", "Choose Mode 2 Calibration TXT", "选择模式二标定 TXT" },
            { "calibration.import_filter", "Calibration text (*.txt);;All files (*.*)", "标定文本 (*.txt);;所有文件 (*.*)" },
            { "calibration.import_error_title", "Calibration TXT Import", "标定 TXT 导入" },
            { "calibration.import_open_error", "Could not open:\n%1", "无法打开：\n%1" },
            { "calibration.import_format_error", "The TXT format is invalid:\n%1", "TXT 格式无效：\n%1" },
            { "calibration.import_success", "Imported Mode 2 calibration and ABB safety point from %1.", "已从 %1 导入模式二标定数据和 ABB 安全点。" },
            { "calibration.operations", "Fit Operations", "拟合操作" },
            { "calibration.clear_selected", "Clear Selected", "清空选中点" },
            { "calibration.clear_mode", "Clear Mode", "清空当前模式" },
            { "calibration.fit_current", "Fit Current Mode", "拟合当前模式" },
            { "calibration.fit_both", "Fit Both", "同时拟合" },
            { "calibration.fit_output", "Fit Results", "拟合结果" },
            { "calibration.show_cylinder", "Show cylinder points and fit", "显示圆柱触碰点与拟合结果" },
            { "calibration.show_circle", "Show circle points and fit", "显示圆触碰点与拟合结果" },
            { "calibration.frame_title", "Base-coordinate Workpiece Pose", "基坐标系下的工件位姿" },
            { "calibration.axis_source", "Axis source", "中心轴线来源" },
            { "calibration.axis_cylinder", "Mode 1: spatial cylinder", "模式一：空间圆柱拟合" },
            { "calibration.axis_circle", "Mode 2: horizontal circle", "模式二：水平圆拟合" },
            { "calibration.reference_points", "Reference Touch Points in Base Coordinates (mm)", "基坐标系参考触碰点（mm）" },
            { "calibration.top_point", "Tooth top", "齿顶点" },
            { "calibration.y_start", "+Y start", "+Y 起点" },
            { "calibration.y_end", "+Y end", "+Y 终点" },
            { "calibration.height", "Actual workpiece height", "工件实际高度" },
            { "calibration.calculate_apply", "Calculate and Apply Workpiece Pose", "计算并应用工件位姿" },
            { "calibration.calculate_tooltip", "Calculate X/Y/Z/Rx/Ry/Rz and write them to the base-coordinate workpiece pose on the left.", "计算 X/Y/Z/Rx/Ry/Rz，并回填到左侧基坐标系下的工件位姿。" },
            { "calibration.load_model_first", "Load a workpiece model before applying the calibrated pose.", "请先加载工件模型，再应用标定位姿。" },
            { "calibration.pose_not_calculated", "Workpiece pose has not been calculated.", "尚未计算工件位姿。" },
            { "calibration.fit_not_ready", "%1: not fitted", "%1：尚未拟合" },
            { "calibration.fit_result", "%1 | R=%2 mm | RMS=%3 mm | max=%4 mm", "%1｜R=%2 mm｜RMS=%3 mm｜最大残差=%4 mm" },
            { "calibration.warning.axial_span", "The axial span is short. Add touch points at more heights.", "轴向跨度较小，建议增加不同高度的触碰点。" },
            { "calibration.warning.short_y", "The +Y baseline is shorter than 50 mm; increase its length to reduce angular error.", "+Y 测量基线短于 50 mm，建议增大间距以降低角度误差。" },
            { "calibration.axis_not_ready", "The selected mode has no valid fitted axis.", "所选模式暂无有效的拟合中心轴线。" },
            { "calibration.axis_result", "Axis point P0=%1 mm\nDirection n=%2", "轴线上一点 P0=%1 mm\n方向 n=%2" },
            { "calibration.error.incomplete_row", "Touch-point row %1 is incomplete.", "第 %1 行触碰点坐标不完整。" },
            { "calibration.error.invalid_number", "Touch-point row %1 contains an invalid number.", "第 %1 行触碰点包含无效数字。" },
            { "calibration.error.cylinder_count", "Mode 1 requires 8-12 complete points; currently %1.", "模式一需要 8~12 个完整触碰点，当前为 %1 个。" },
            { "calibration.error.circle_count", "Mode 2 requires exactly 6 complete points; currently %1.", "模式二需要 6 个完整触碰点，当前为 %1 个。" },
            { "calibration.error.circle_plane", "The six circle points exceed the 0.1 mm Z-plane tolerance.", "6 个圆触碰点未满足 0.1 mm 的 Z 平面容差。" },
            { "calibration.error.fit_failed", "The fit failed because the touch points are degenerate or unstable.", "触碰点退化或分布不稳定，拟合失败。" },
            { "calibration.error.axis_missing", "Fit the selected center-axis mode first.", "请先完成所选中心轴线模式的拟合。" },
            { "calibration.error.top_missing", "Enter the complete tooth-top point X/Y/Z.", "请输入完整的齿顶点 X/Y/Z。" },
            { "calibration.error.y_start_missing", "Enter the complete +Y start point X/Y/Z.", "请输入完整的 +Y 起点 X/Y/Z。" },
            { "calibration.error.y_end_missing", "Enter the complete +Y end point X/Y/Z.", "请输入完整的 +Y 终点 X/Y/Z。" },
            { "calibration.error.y_short", "The +Y start and end points must be at least 1 mm apart.", "+Y 起点与终点的距离必须至少为 1 mm。" },
            { "calibration.error.y_parallel", "The measured +Y direction is too close to the rotary axis.", "测得的 +Y 方向与回转轴过于平行。" },
            { "calibration.error.frame_failed", "The workpiece frame could not be calculated from the current inputs.", "无法根据当前输入计算工件坐标系。" },
            { "calibration.pose_result", "Applied pose\nXYZ=%1 mm\nRx=%2 deg  Ry=%3 deg  Rz=%4 deg\nABB q=[%5, %6, %7, %8]", "已应用位姿\nXYZ=%1 mm\nRx=%2°  Ry=%3°  Rz=%4°\nABB q=[%5, %6, %7, %8]" },
            { "trajectory.generation", "Trajectory Generation", "轨迹生成" },
            { "trajectory.spray_distance", "Local +Z spray distance", "局部 +Z 喷涂距离" },
            { "trajectory.tilt", "Trajectory tilt", "轨迹倾斜角度" },
            { "trajectory.speed", "Travel speed", "轨迹移动速度" },
            { "trajectory.start_extension", "Start extension", "起点喷涂延伸距离" },
            { "trajectory.end_extension", "End extension", "终点喷涂延伸距离" },
            { "trajectory.point_count", "Point count", "轨迹点数量" },
            { "trajectory.positioner_rpm", "Positioner speed", "变位机转速" },
            { "trajectory.auto_points", "Auto (max. 1 mm)", "自动（间距不大于 1 mm）" },
            { "trajectory.reopen_boundary", "Reconfirm Region", "重新确认框图" },
            { "trajectory.generate", "Generate Points", "确认生成轨迹点" },
            { "trajectory.swap", "Swap Start / End", "交换起止点" },
            { "trajectory.metrics", "Trajectory Summary", "轨迹信息" },
            { "trajectory.metric_count", "Point count", "轨迹点数量" },
            { "trajectory.metric_duration", "Motion time", "轨迹运动时间" },
            { "trajectory.metric_interval", "Trajectory point interval", "轨迹点时间间隔" },
            { "trajectory.points", "Trajectory Poses", "轨迹点列表" },
            { "trajectory.interpolation_interval", "Interpolation interval", "插值时间" },
            { "trajectory.interpolate", "Apply Interpolation to Trajectory", "应用插值到轨迹" },
            { "trajectory.show_helical", "Show Helical Trajectory", "显示螺旋轨迹" },
            { "trajectory.show_linear", "Show Linear Trajectory", "显示直线轨迹" },
            { "trajectory.point_transform", "Selected Pose Adjustment", "所选轨迹点平移与旋转" },
            { "trajectory.apply_transform", "Apply to Selected Poses", "应用到所选轨迹点" },
            { "trajectory.group", "Trajectory Group", "轨迹组" },
            { "trajectory.new", "New", "新建轨迹" },
            { "trajectory.edit", "Edit", "编辑轨迹" },
            { "trajectory.remove", "Remove", "删除轨迹" },
            { "trajectory.save_to_group", "Save to Group", "保存到轨迹组" },
            { "trajectory.export_group", "Save Trajectory Group", "保存轨迹组文件" },
            { "trajectory.transition_after", "Interval to next trajectory", "到下一条轨迹的时间间隔" },
            { "trajectory.pass_summary", "Trajectory %1 | %2 points | %3 s", "轨迹 %1 | %2 点 | %3 秒" },
            { "abb.settings", "ABB RAPID Settings", "ABB RAPID 设置" },
            { "abb.safety_x", "Safety point X", "安全点 X" },
            { "abb.safety_y", "Safety point Y", "安全点 Y" },
            { "abb.safety_z", "Safety point Z", "安全点 Z" },
            { "abb.safety_speed", "Safety move speed", "安全点移动速度" },
            { "abb.module_name", "Module name", "模块名" },
            { "abb.file_name", ".mod file name", ".mod 文件名" },
            { "abb.tool_name", "Tool data name", "工具坐标系名称" },
            { "abb.output_directory", "Output directory", "保存目录" },
            { "abb.browse", "Browse", "浏览" },
            { "abb.output_dialog", "Choose ABB RAPID Output Directory", "选择 ABB RAPID 保存目录" },
            { "abb.sequence", "Instruction Assembly Order", "指令拼接顺序" },
            { "abb.add_safety", "Add Safety Point", "添加安全点" },
            { "abb.add_trajectory", "Add Trajectory", "添加轨迹" },
            { "abb.move_up", "Move Up", "上移" },
            { "abb.move_down", "Move Down", "下移" },
            { "abb.remove", "Remove", "移除" },
            { "abb.generate", "Generate and Save Both .mod Files", "生成并保存两种 .mod" },
            { "abb.safety_item", "MoveJ | Safety point", "MoveJ | 安全点" },
            { "abb.trajectory_item", "Trajectory %1 | MoveJ start + MoveL end", "轨迹 %1 | MoveJ 起点 + MoveL 终点" },
            { "abb.missing_trajectory", "Missing trajectory", "轨迹已不存在" },
            { "abb.generated_preview", "Generated Instruction Preview", "生成后逐指令预览" },
            { "abb.previous", "Previous", "上一条" },
            { "abb.reset", "Reset", "复位" },
            { "abb.next", "Next", "下一条" },
            { "abb.preview_safety", "Safety point", "安全点" },
            { "abb.preview_not_generated", "No .mod file has been generated in this session.", "本次尚未生成 .mod 文件。" },
            { "abb.saved_to", "Saved both files to:\n%1", "已保存两份文件：\n%1" },
            { "abb.export_failed", "Save failed: %1", "保存失败：%1" },
            { "command.return_to_workpiece", "View Transform", "视角变换" },
            { "command.return_to_workpiece_tooltip", "Switch back to the isolated planning workpiece view after inspecting another scene item.", "在查看其他场景对象后，切换回规划工件视角。" },
            { "model.planning_object", "Planning Object", "规划对象" },
            { "model.complete_part", "Complete Part", "完整件规划" },
            { "model.simulation_block", "Simulation Block", "模拟块规划" },
            { "model.load", "Load Model", "加载模型" },
            { "model.load_tooltip", "Choose a workpiece model to import and align.", "选择要导入并自动摆正的工件模型。" },
            { "model.dialog_title", "Load Rotation-body Workpiece", "加载回转体工件" },
            { "model.dialog_filter", "Object Meshes (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY);;STL Files (*.stl *.STL);;All Files (*.*)", "工件网格 (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY);;STL 文件 (*.stl *.STL);;所有文件 (*.*)" },
            { "model.no_source", "No model loaded", "尚未加载模型" },
            { "simulation.maximum_diameter", "Mother maximum diameter", "母体最大直径" },
            { "simulation.rotation_axis", "Original rotation axis", "原始回转轴方向" },
            { "simulation.tooth_outward", "Tooth outward direction", "齿顶朝外方向" },
            { "simulation.axis_tooltip", "The rotation axis and tooth outward direction must be perpendicular.", "回转轴方向与齿顶朝外方向必须互相垂直。" },
            { "simulation.axes_invalid", "Choose two perpendicular directions before loading.", "加载前请选择两个互相垂直的方向。" },
            { "statistics.title", "Model Information", "模型信息" },
            { "statistics.height", "Height", "高度" },
            { "statistics.maximum_diameter", "Maximum diameter", "最大直径" },
            { "statistics.minimum_diameter", "Minimum diameter", "最小直径" },
            { "statistics.bounds", "Bounding size X / Y / Z", "包围盒尺寸 X / Y / Z" },
            { "statistics.axis", "Estimated axis", "估计回转轴" },
            { "statistics.unavailable", "--", "--" },
            { "alignment.title", "Automatic Alignment", "自动摆正" },
            { "alignment.no_model", "Load a model to calculate its planning frame.", "加载模型后计算规划坐标系。" },
            { "alignment.ready", "Aligned. Verify the frame before sectioning.", "已自动摆正，请在剖切前确认坐标系。" },
            { "alignment.flip", "Flip Workpiece", "翻转工件" },
            { "alignment.flip_tooltip", "Exchange the two axial ends and place the new bottom on Z=0.", "交换回转轴两端，并将新底面重新贴合 Z=0。" },
            { "alignment.reset", "Restore Auto Alignment", "恢复自动摆正" },
            { "alignment.reset_tooltip", "Return to the automatic alignment baseline.", "恢复到本次加载后的自动摆正基线。" },
            { "transform.adjustment", "Workpiece Translation and Rotation", "工件平移与旋转" },
            { "transform.base_frame", "Workpiece Frame Pose in Base Coordinates", "工件坐标系在基坐标系中的位姿" },
            { "transform.length_unit", "mm", "mm" },
            { "transform.angle_unit", "deg", "度" },
            { "publish.title", "Workpiece Position after Exiting", "退出模块后的工件位置" },
            { "publish.base", "Use Base-coordinate Pose", "按基坐标系位姿放置" },
            { "publish.local", "Place at Planning Origin", "放在规划原点" },
            { "publish.confirm_frame", "Confirm Workpiece Frame", "确认工件坐标系" },
            { "publish.confirm_frame_tooltip", "Confirm T_PM and enable YZ section extraction.", "确认 T_PM，并启用 YZ 剖切。" },
            { "section.view_title", "Main View", "主视图" },
            { "section.scene_3d", "3D Scene", "3D 场景" },
            { "section.section_view", "Section", "剖面" },
            { "section.preview", "YZ Section Preview", "YZ 剖面预览" },
            { "section.extract", "Extract Section", "执行剖切" },
            { "section.reextract", "Re-extract Section", "重新剖切" },
            { "section.extract_tooltip", "Intersect the workpiece with planning plane X=0 and select the Y>0 outer contour.", "使用规划平面 X=0 剖切工件，并选择 Y>0 的目标外轮廓。" },
            { "section.empty", "No YZ section", "暂无 YZ 剖面" },
            { "region.title", "Spray Region Classification", "喷涂区域分类" },
            { "region.auto_recognize", "Auto Recognize", "自动识别" },
            { "region.auto_tooltip", "Classify tooth top, wall, bottom and transition segments.", "自动识别齿顶、齿壁、齿底和过渡区。" },
            { "region.active_label", "Manual label", "人工框选标签" },
            { "region.unclassified", "Unclassified", "未分类" },
            { "region.tooth_top", "Tooth Top", "齿顶" },
            { "region.tooth_wall", "Tooth Wall", "齿壁" },
            { "region.tooth_bottom", "Tooth Bottom", "齿底" },
            { "region.transition", "Transition", "过渡区" },
            { "region.edit_title", "Manual Correction", "人工修正" },
            { "region.undo", "Undo", "撤销" },
            { "region.undo_tooltip", "Undo the last manual region correction.", "撤销上一次人工区域修正。" },
            { "region.redo", "Redo", "重做" },
            { "region.redo_tooltip", "Redo the next manual region correction.", "重做下一次人工区域修正。" },
            { "region.restore", "Restore Automatic", "恢复自动结果" },
            { "region.restore_tooltip", "Remove all manual overrides and restore automatic labels.", "清除全部人工覆盖并恢复自动识别标签。" },
            { "boundary.title", "Spray Region Boundary", "喷涂区域边界" },
            { "boundary.maximum_y", "Maximum Tooth Top Y", "最大齿顶 Y" },
            { "boundary.envelope", "Tooth-top Fitted Envelope", "齿顶拟合斜边" },
            { "boundary.confirm", "Confirm Spray Region", "确认喷涂区域" },
            { "boundary.confirm_tooltip", "Build the yellow boundary from the four spray region classes.", "根据四类喷涂区域生成黄色边界。" },
            { "command.save", "Save Progress && Exit", "保存进度并退出" },
            { "command.save_tooltip", "Save the adjusted workpiece and private planning progress, then exit.", "保存调整后的工件和本模块规划进度，然后退出。" },
            { "command.discard", "Discard && Exit", "放弃并退出" },
            { "command.discard_tooltip", "Restore the project state from module entry and exit.", "恢复进入模块前的项目状态，然后退出。" },
            { "status.returned_to_workpiece", "Switched to the planning workpiece view.", "已切换至规划工件视角。" },
            { "status.flipped", "The workpiece was flipped and the new bottom was placed on Z=0.", "已翻转工件，并将新底面贴合 Z=0。" },
            { "status.alignment_reset", "The automatic alignment pose was restored.", "已恢复自动摆正结果。" },
            { "status.calibration_applied", "The calibrated workpiece pose was applied to the base-coordinate fields.", "标定得到的工件位姿已回填到基坐标系位姿输入框。" },
            { "status.saved", "Rotation-body planning progress saved.", "回转体规划进度已保存。" },
            { "status.saved_view_refresh_failed", "Planning data was saved, but the viewport refresh failed. Please check the view.", "规划数据已保存，但视口刷新失败，请检查视图。" },
            { "status.discarded", "Rotation-body planning changes discarded.", "已放弃回转体规划修改。" },
            { "status.restored", "Rotation-body planning progress restored.", "已恢复回转体规划进度。" },
            { "stage.no_model", "No model", "未加载模型" },
            { "stage.model_loaded", "Model loaded", "模型已加载" },
            { "stage.frame_confirmed", "Frame confirmed", "坐标系已确认" },
            { "stage.section_ready", "Section ready", "剖面已生成" },
            { "stage.regions_ready", "Regions ready", "区域已确认" },
            { "stage.boundary_confirmed", "Spray boundary confirmed", "喷涂边界已确认" },
            { "error.none", "", "" },
            { "error.invalid_argument", "Check the current parameters and try again.", "请检查当前参数后重试。" },
            { "error.empty_mesh", "Load a valid model before continuing.", "请先加载有效模型。" },
            { "error.non_finite", "The model contains invalid numeric geometry.", "模型包含无效数值几何。" },
            { "error.invalid_axis", "Choose two perpendicular axis directions.", "请选择两个互相垂直的轴向。" },
            { "error.alignment", "Automatic alignment failed; verify the model geometry.", "自动摆正失败，请检查模型几何。" },
            { "error.section", "The YZ section could not be extracted.", "无法生成 YZ 剖面。" },
            { "error.no_contour", "No valid Y>0 outer contour was found.", "未找到有效的 Y>0 外轮廓。" },
            { "error.region", "There is not enough valid contour data for region planning.", "有效轮廓数据不足，无法进行区域规划。" },
            { "error.tooth_top", "There are not enough tooth-top points for this boundary mode.", "齿顶点不足，无法使用当前边界模式。" },
            { "error.singular_fit", "The tooth-top envelope could not be fitted reliably.", "无法可靠拟合齿顶外包络。" },
            { "error.generic", "The operation could not be completed.", "操作未能完成。" }
        };

        bool isChinese(const QString& languageCode)
        {
            return RotationBodyPlanningTranslations::canonicalLanguageCode(languageCode) ==
                QStringLiteral("zh-CN");
        }
    }

    QString RotationBodyPlanningTranslations::canonicalLanguageCode(const QString& languageCode)
    {
        QString normalized = languageCode.trimmed().toLower();
        normalized.replace(QLatin1Char('_'), QLatin1Char('-'));
        return normalized == QStringLiteral("zh") || normalized.startsWith(QStringLiteral("zh-"))
            ? QStringLiteral("zh-CN")
            : QStringLiteral("en");
    }

    QString RotationBodyPlanningTranslations::text(
        const QString& languageCode,
        const char* key)
    {
        if(key == nullptr) {
            return {};
        }
        for(const TranslationEntry& entry : entries) {
            if(QLatin1String(entry.key) == QLatin1String(key)) {
                return QString::fromUtf8(isChinese(languageCode) ? entry.chinese : entry.english);
            }
        }
        return QString::fromLatin1(key);
    }

    QString RotationBodyPlanningTranslations::stageName(
        const QString& languageCode,
        smrobot::spray::rotationbody::PlanningStage stage)
    {
        using Stage = smrobot::spray::rotationbody::PlanningStage;
        switch(stage) {
        case Stage::NoModel: return text(languageCode, "stage.no_model");
        case Stage::ModelLoaded: return text(languageCode, "stage.model_loaded");
        case Stage::FrameConfirmed: return text(languageCode, "stage.frame_confirmed");
        case Stage::SectionReady: return text(languageCode, "stage.section_ready");
        case Stage::RegionsReady: return text(languageCode, "stage.regions_ready");
        case Stage::SprayBoundaryConfirmed: return text(languageCode, "stage.boundary_confirmed");
        }
        return text(languageCode, "stage.no_model");
    }

    QString RotationBodyPlanningTranslations::axisName(
        const QString&,
        smrobot::spray::rotationbody::SignedAxis axis)
    {
        using Axis = smrobot::spray::rotationbody::SignedAxis;
        switch(axis) {
        case Axis::PositiveX: return QStringLiteral("+X");
        case Axis::NegativeX: return QStringLiteral("-X");
        case Axis::PositiveY: return QStringLiteral("+Y");
        case Axis::NegativeY: return QStringLiteral("-Y");
        case Axis::PositiveZ: return QStringLiteral("+Z");
        case Axis::NegativeZ: return QStringLiteral("-Z");
        }
        return QStringLiteral("+Z");
    }

    QString RotationBodyPlanningTranslations::regionName(
        const QString& languageCode,
        smrobot::spray::rotationbody::RegionLabel label)
    {
        using Label = smrobot::spray::rotationbody::RegionLabel;
        switch(label) {
        case Label::Unclassified: return text(languageCode, "region.unclassified");
        case Label::ToothTop: return text(languageCode, "region.tooth_top");
        case Label::ToothWall: return text(languageCode, "region.tooth_wall");
        case Label::ToothBottom: return text(languageCode, "region.tooth_bottom");
        case Label::Transition: return text(languageCode, "region.transition");
        }
        return text(languageCode, "region.unclassified");
    }

    QString RotationBodyPlanningTranslations::errorText(
        const QString& languageCode,
        smrobot::spray::rotationbody::PlanningErrorCode code)
    {
        using Code = smrobot::spray::rotationbody::PlanningErrorCode;
        switch(code) {
        case Code::None: return text(languageCode, "error.none");
        case Code::InvalidArgument:
        case Code::InvalidTriangleIndex:
        case Code::DegenerateGeometry:
            return text(languageCode, "error.invalid_argument");
        case Code::EmptyMesh: return text(languageCode, "error.empty_mesh");
        case Code::NonFiniteGeometry: return text(languageCode, "error.non_finite");
        case Code::InvalidAxisSelection: return text(languageCode, "error.invalid_axis");
        case Code::AlignmentFailed: return text(languageCode, "error.alignment");
        case Code::SectionFailed: return text(languageCode, "error.section");
        case Code::NoContour: return text(languageCode, "error.no_contour");
        case Code::InsufficientRegionData: return text(languageCode, "error.region");
        case Code::InsufficientToothTopData: return text(languageCode, "error.tooth_top");
        case Code::SingularFit: return text(languageCode, "error.singular_fit");
        }
        return text(languageCode, "error.generic");
    }
}
